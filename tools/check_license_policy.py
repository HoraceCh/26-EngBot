#!/usr/bin/env python3
"""Validate the repository's licensing inventory and release blockers.

This tool is deliberately read-only. It does not infer or grant licenses and
does not alter source headers. Use the default report mode during development;
use --enforce before preparing a public release.
"""

from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
POLICY_FILE = REPOSITORY_ROOT / "THIRD_PARTY_COMPONENTS.json"
VALID_STATUSES = {"cleared", "conditional", "blocked"}


def relative_posix(path: Path) -> str:
    return path.resolve().relative_to(REPOSITORY_ROOT).as_posix()


def expand_pattern(pattern: str) -> list[str]:
    absolute_pattern = str(REPOSITORY_ROOT / Path(pattern))
    matches = {
        relative_posix(Path(match))
        for match in glob.glob(absolute_pattern, recursive=True)
        if Path(match).exists()
    }
    return sorted(matches)


def load_policy() -> dict[str, Any]:
    with POLICY_FILE.open("r", encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise ValueError("policy root must be a JSON object")
    return value


def validate_structure(policy: dict[str, Any]) -> list[str]:
    errors: list[str] = []

    if policy.get("schema_version") != 1:
        errors.append("schema_version must be 1")

    profiles = policy.get("release_profiles")
    if not isinstance(profiles, dict) or not profiles:
        errors.append("release_profiles must be a non-empty object")
        profiles = {}

    for required in policy.get("required_files", []):
        if not (REPOSITORY_ROOT / required).is_file():
            errors.append(f"required file is missing: {required}")

    seen_ids: set[str] = set()
    components = policy.get("components")
    if not isinstance(components, list) or not components:
        errors.append("components must be a non-empty array")
        return errors

    for index, component in enumerate(components):
        label = f"components[{index}]"
        if not isinstance(component, dict):
            errors.append(f"{label} must be an object")
            continue

        component_id = component.get("id")
        if not isinstance(component_id, str) or not component_id:
            errors.append(f"{label}.id must be a non-empty string")
        elif component_id in seen_ids:
            errors.append(f"duplicate component id: {component_id}")
        else:
            seen_ids.add(component_id)

        status = component.get("status")
        if status not in VALID_STATUSES:
            errors.append(
                f"{component_id or label}: status must be one of "
                f"{sorted(VALID_STATUSES)}"
            )

        paths = component.get("paths")
        if not isinstance(paths, list) or not paths:
            errors.append(f"{component_id or label}: paths must be non-empty")
        else:
            for pattern in paths:
                if not isinstance(pattern, str) or not pattern:
                    errors.append(
                        f"{component_id or label}: path patterns must be strings"
                    )
                elif not expand_pattern(pattern):
                    errors.append(
                        f"{component_id or label}: path pattern matched nothing: "
                        f"{pattern}"
                    )

        evidence = component.get("evidence")
        if not isinstance(evidence, list) or not evidence:
            errors.append(f"{component_id or label}: evidence must be non-empty")
        else:
            for evidence_path in evidence:
                if not (REPOSITORY_ROOT / evidence_path).is_file():
                    errors.append(
                        f"{component_id or label}: evidence is missing: "
                        f"{evidence_path}"
                    )

        blocks = component.get("blocks")
        if not isinstance(blocks, list):
            errors.append(f"{component_id or label}: blocks must be an array")
        else:
            unknown_profiles = set(blocks) - set(profiles)
            if unknown_profiles:
                errors.append(
                    f"{component_id or label}: unknown release profiles: "
                    f"{sorted(unknown_profiles)}"
                )
            if status != "blocked" and blocks:
                errors.append(
                    f"{component_id or label}: only blocked components may "
                    "declare blocking profiles"
                )

        if not component.get("required_action"):
            errors.append(f"{component_id or label}: required_action is missing")

    return errors


def scan_restricted_notices(policy: dict[str, Any]) -> tuple[list[str], list[str]]:
    scan = policy.get("restricted_notice_scan", {})
    needle = str(scan.get("needle", "")).casefold()
    suffixes = {str(item).casefold() for item in scan.get("suffixes", [])}
    known = {str(item).replace("\\", "/") for item in scan.get("known_files", [])}
    found: set[str] = set()

    for root_text in scan.get("roots", []):
        root = REPOSITORY_ROOT / root_text
        if not root.is_dir():
            continue
        for candidate in root.rglob("*"):
            if not candidate.is_file() or candidate.suffix.casefold() not in suffixes:
                continue
            try:
                contents = candidate.read_text(encoding="utf-8", errors="ignore")
            except OSError:
                continue
            if needle and needle in contents.casefold():
                found.add(relative_posix(candidate))

    return sorted(found - known), sorted(known - found)


def blockers_for_profile(
    policy: dict[str, Any], profile: str
) -> list[dict[str, str]]:
    blockers: list[dict[str, str]] = []
    for component in policy["components"]:
        if component.get("status") != "blocked":
            continue
        if profile not in component.get("blocks", []):
            continue
        blockers.append(
            {
                "id": component["id"],
                "name": component["name"],
                "license_expression": component["license_expression"],
                "required_action": component["required_action"],
            }
        )
    return blockers


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        default="repository-public",
        help="release profile to assess (default: repository-public)",
    )
    parser.add_argument(
        "--enforce",
        action="store_true",
        help="return exit code 1 when unresolved release blockers exist",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="emit the result as JSON",
    )
    args = parser.parse_args()

    try:
        policy = load_policy()
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"license policy error: {error}", file=sys.stderr)
        return 2

    profiles = policy.get("release_profiles", {})
    if args.profile not in profiles:
        print(
            f"unknown release profile {args.profile!r}; choose one of "
            f"{', '.join(sorted(profiles))}",
            file=sys.stderr,
        )
        return 2

    errors = validate_structure(policy)
    unknown_restricted, stale_restricted = scan_restricted_notices(policy)
    if unknown_restricted:
        errors.extend(
            f"unrecorded restricted notice: {path}" for path in unknown_restricted
        )
    if stale_restricted:
        errors.extend(
            f"recorded restricted notice no longer found: {path}"
            for path in stale_restricted
        )

    blockers = blockers_for_profile(policy, args.profile)
    result = {
        "policy": relative_posix(POLICY_FILE),
        "profile": args.profile,
        "inventory_date": policy.get("inventory_date"),
        "component_count": len(policy.get("components", [])),
        "metadata_errors": errors,
        "release_blockers": blockers,
        "release_ready": not errors and not blockers,
    }

    if args.json:
        print(json.dumps(result, indent=2, ensure_ascii=False))
    else:
        print(
            f"License inventory: {result['component_count']} components; "
            f"profile={args.profile}"
        )
        if errors:
            print(f"Metadata errors ({len(errors)}):")
            for error in errors:
                print(f"  - {error}")
        else:
            print("Metadata errors: 0")

        if blockers:
            print(f"Release blockers ({len(blockers)}):")
            for blocker in blockers:
                print(
                    f"  - {blocker['id']}: {blocker['name']} "
                    f"[{blocker['license_expression']}]"
                )
                print(f"    Required: {blocker['required_action']}")
        else:
            print("Release blockers: 0")

        print(
            "Decision: "
            + (
                "READY for this profile"
                if result["release_ready"]
                else "BLOCKED for this profile"
            )
        )

    if errors:
        return 2
    if args.enforce and blockers:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
