# 许可证、版权与第三方代码审计

> 审计日期：2026-07-23；基线提交 `d7aa4214765a36a291678e58afd736484ac28083`。  
> 本文是仓库内声明的技术盘点，不是最终法律意见。公开可访问不等于具有开源许可证；文件级声明优先于仓库级说明。

## 0. 本次治理修复状态

针对“混合许可边界虽已声明、但缺少逐组件清单和可执行发布门禁”的问题，仓库已完成以下可在不改变任何权利归属前提下实施的修复：

- 新增根目录 `THIRD_PARTY_COMPONENTS.json`，登记 15 类项目/第三方组件的路径、权利人、许可表达式、直接证据、状态、受阻发布范围和解除条件。
- 新增只读检查器 `tools/check_license_policy.py`。它验证清单结构、证据文件、路径匹配和活动业务源码中的 `all rights reserved` 声明，并按 `repository-public`、`firmware-source`、`firmware-binary`、`mechanical` 四种发布范围给出阻断结果。
- 根 `LICENSE`、`NOTICE` 和 `README.md` 已加入清单与发布门禁入口，明确禁止通过删除版权头或擅自添加项目许可证来绕过阻断。
- 2026-07-23 校验结果：15 个组件记录、0 个清单结构错误；`repository-public` 仍有 8 类发布阻断项。此处“不再静默或误授权”已经修复；HNU/命名作者/厂商材料的实际授权本身仍须由权利人、原始发行包证据或合法替换来解决。【已确认｜A级】

常规盘点（不因阻断返回失败）：

```bash
python tools/check_license_policy.py
```

发布门禁：

```bash
python tools/check_license_policy.py --profile repository-public --enforce
```

退出码 `1` 表示存在已登记发布阻断，退出码 `2` 表示清单与仓库不一致。未经书面授权，不应把这两类结果改成“已解决”。

## 1. 仓库许可结构

根 `LICENSE` 明确声明这是 mixed-source repository，而非一个不可分割、统一许可的作品：

- 有权授权的项目原创固件贡献：`Firmware/LICENSE` 指向 Apache-2.0；完整文本为 `LICENSES/Apache-2.0.txt`。
- 有权授权的项目原创机械设计：`Mechanical/LICENSE` 指向 CERN-OHL-P-2.0；完整文本为 `LICENSES/CERN-OHL-P-2.0.txt`。
- 自带声明的第三方、`all rights reserved` 文件、厂商模型/手册、来源或授权权限未确认的材料：不被上述项目许可重新授权。
- `NOTICE` 已列出 CMSIS、ST HAL/USB、FreeRTOS、SEGGER、HNU YueLu EC、厂商资料/模型等主要类别，并要求保留声明。

这套顶层说明正确表达了“只在贡献者确有权利的范围内授权”，但不能替代逐文件来源证明、贡献者授权和发布清单。【已确认｜A级】

## 2. 组件审计表

| 组件 | 路径 | 版权方/来源 | 许可证或版权声明 | 允许再分发 | 允许修改 | 风险 |
| --- | --- | --- | --- | --- | --- | --- |
| 项目原创固件贡献 | `Firmware/` 中无更具体声明且贡献者有权授权的部分 | 26-EngBot contributors | Apache-2.0；`Firmware/LICENSE` | 是，须满足 Apache 条件 | 是 | 中：必须先证明具体文件属于授权范围 |
| 项目原创机械设计 | `Mechanical/` 中满足范围的原创文件 | 26-EngBot contributors | CERN-OHL-P-2.0；`Mechanical/LICENSE` | 是，须保留/补充修改通知 | 是 | 中：供应商/RoboMaster 模型被明确排除 |
| STM32F4 HAL/LL | 两工程 `Drivers/STM32F4xx_HAL_Driver/` | STMicroelectronics | 随附 `LICENSE.txt`：包许可优先；缺包条款时 BSD-3-Clause | 条件允许 | 条件允许 | 中：仓库未见所称 `Package_license`，需确认原始软件包条款 |
| Arm CMSIS Core/DSP | 两工程 `Drivers/CMSIS/`、`Middlewares/ST/ARM/DSP/` | Arm Limited 等 | `Drivers/CMSIS/LICENSE.txt` 为 Apache-2.0；源文件另有声明 | 通常允许并保留声明 | 通常允许 | 低至中：预编译 DSP 库需核对对应发行包条款 |
| ST CMSIS Device | `Drivers/CMSIS/Device/ST/STM32F4xx/` | STMicroelectronics | `LICENSE.txt`：包许可优先；缺包条款时 Apache-2.0 | 条件允许 | 条件允许 | 中：同样缺 `Package_license` 上下文 |
| FreeRTOS | `Middlewares/Third_Party/FreeRTOS/Source/` | Amazon.com, Inc. or affiliates | 随附 MIT 文本（虽含“All Rights Reserved”，随后给出 MIT 权限） | 是，保留版权和许可 | 是 | 低 |
| CMSIS-RTOS v1 适配 | FreeRTOS `Source/CMSIS_RTOS/` | Arm/ST/FreeRTOS 分发组件 | 文件头/随附组件许可 | 条件允许 | 条件允许 | 低至中：随发行包一并保留声明 |
| STM32 USB Device Library | `Middlewares/ST/STM32_USB_Device_Library/` | STMicroelectronics | SLA0044；随附全文 | 条件允许；二进制/源代码有保留条件 | 条件允许但仅限 ST 设备组合 | 高：条款 4 的 ST 设备限制和条款 5 的 Open Source Terms 限制必须单列审查 |
| SEGGER RTT | `Middlewares/Third_Party/SEGGER/` | SEGGER Microcontroller GmbH | 文件头允许源/二进制再分发，须保留版权、条件和免责声明 | 是，满足文件头条件 | 文字上允许，但 SEGGER 建议不修改 | 中：没有独立 LICENSE 文件，必须保留每个源文件完整头 |
| HNU YueLu EC 代码 | `application/robot_def.h`、`modules/algorithm/controller.c`、`modules/motor/DJImotor/dji_motor.h` 等 | HNU YueLu EC | `all rights reserved`，未见附加开放许可 | 不能从仓库证据确认 | 不能从仓库证据确认 | 高：根 Apache 声明明确不覆盖；需权利人书面授权 |
| Wang Hongxi 算法 | `modules/algorithm/QuaternionEKF.*`、`kalman_filter.*` | 文件头作者 Wang Hongxi | 作者/版本/日期，无明确 SPDX/许可 | 未确认 | 未确认 | 高：活动 INS 使用相关代码，发布前需来源和授权链 |
| DM/RS/HT/YS/LK/DJI 电机驱动 | `modules/motor/` | 混合：项目、HNU、厂商/未知 | 部分有版权头，部分无明确许可 | 未能整体确认 | 未能整体确认 | 高：应逐文件建立来源、协议依据和授权 |
| BMI088/编码器/遥控器等驱动 | `modules/imu/`、`modules/encoder/`、`modules/remote/` | Bosch/作者/项目或未知 | 头注释不统一；未见目录级专属许可 | 未能整体确认 | 未能整体确认 | 高：驱动代码与数据手册权利需区分 |
| RoboMaster 裁判/通信资料 | `Doc/`、`manual/`、模块协议定义 | DJI/RoboMaster 等 | 厂商资料/协议声明；不由项目许可覆盖 | 取决于原始条款 | 通常不适用于文档 | 高：公开赛事资料不等于可任意再分发 |
| 厂商数据手册与 PDF | 仓库 `Doc/`、`manual/` 及其他 PDF | 各设备制造商 | 文件自身版权/网站条款 | 未确认 | 通常不允许改写原件 | 高：发布包应按来源和必要性筛选 |
| 预编译 CMSIS-DSP 库 | `Middlewares/ST/ARM/DSP/Lib/*.a` | Arm/ST | 相关发行包条款 | 需核对 | 二进制通常不可修改 | 中至高：源码许可不能自动推定二进制分发条件 |
| CubeMX 生成代码 | 两工程 `Src/Inc/Core/USB_DEVICE` 中生成区域 | ST 模板 + 项目配置贡献 | 文件头/ST 条款，且生成结果包含项目配置 | 条件允许 | USER CODE 区通常可修改 | 中：不能简单全标项目 Apache |
| 构建产物 | 两工程 `build/*.elf/.hex/.bin/.map/...` | 所有链接组件的汇总 | 受输入组件全部条款约束 | 需逐组件满足 | 不适用 | 高：没有可验证的源码提交/工具链/NOTICE 绑定记录 |

## 3. 关键许可证依据

| 路径 | 直接证据 | 结论 | 可信度 |
| --- | --- | --- | --- |
| `LICENSE` | “not licensed as one indivisible work”；列出授权范围和排除项 | 仓库不是单一 Apache 项目 | 【已确认｜A级】 |
| `NOTICE` | 列出主要第三方类别并要求保留声明 | 已有基本归属导航 | 【已确认｜A级】 |
| `Firmware/LICENSE` | 只覆盖贡献者有权授权的原创固件，排除第三方和 ARR | 不能覆盖 HNU/厂商/未知来源文件 | 【已确认｜A级】 |
| `Drivers/CMSIS/LICENSE.txt` | Apache-2.0 全文 | CMSIS 目录具有明确随附文本 | 【已确认｜A级】 |
| `Drivers/STM32F4xx_HAL_Driver/LICENSE.txt` | Package_license 优先；否则 BSD-3-Clause | 仍需找到原发行包条款 | 【已确认｜A级】（文件内容）；最终适用条款【代码中未确认｜D级】 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/LICENSE.txt` | Package_license 优先；否则 Apache-2.0 | 同上 | 【已确认｜A级】/【代码中未确认｜D级】 |
| `Middlewares/Third_Party/FreeRTOS/Source/LICENSE` | MIT 许可条件 | 可在保留声明条件下使用/修改/分发 | 【已确认｜A级】 |
| `Middlewares/ST/STM32_USB_Device_Library/LICENSE.txt` | SLA0044 全文，条款 4/5 有特定限制 | 不是普通宽松开源许可 | 【已确认｜A级】 |
| `SEGGER_RTT.c` 等 | 文件头给出再分发条件 | 必须逐文件保留头部 | 【已确认｜A级】 |
| HNU 文件头 | `all rights reserved` | 不能默认重新许可 | 【已确认｜A级】 |

## 4. 主要风险与需要确认的问题

### 高风险

1. **活动控制代码的权利链不完整。** `controller.c` 和 `robot_def.h` 等带 HNU YueLu EC `all rights reserved`；根许可明确没有覆盖它们。仓库可访问并不提供修改或再分发授权。
2. **命名作者算法无明确许可。** Quaternion EKF/Kalman 文件有作者但无 SPDX/许可，且活动 INS 依赖它们。
3. **厂商驱动和协议实现来源不统一。** 多类电机/传感器代码没有可核对的下载地址、发行包版本或授权文本。
4. **USB SLA0044 具有使用范围限制。** 它不能被概括为 Apache/MIT/BSD，移植非 ST 芯片或组合分发需专项确认。
5. **资料与二进制不等同源码许可。** 数据手册、协议 PDF、CAD 模型和 `.a`/固件产物必须分别确认。

### 需要原作者或律师确认

- HNU YueLu EC 是否授权本项目修改、公开、二进制分发和再许可；授权主体是否确为权利人。
- Wang Hongxi 算法文件的原始项目、版本、许可证和项目内修改历史。
- 各 DM/RS/HT/YS/LK/DJI/编码器/BMI088 驱动是团队原创、厂商 SDK 还是网络来源。
- STM32Cube 软件包的 `Package_license` 是否随原始包存在，以及与仓库中版本一致。
- SLA0044 与仓库 Apache-2.0 部分的组合/分发方式是否满足条款 5；本文不作兼容性法律结论。
- 预编译 CMSIS-DSP `.a`、厂商 PDF、RoboMaster 材料和供应商 CAD 是否需要从公开发布包移除或改为链接。

## 5. 发布前最低清单

1. 以现有 `THIRD_PARTY_COMPONENTS.json` 为组件级基线，继续生成逐文件 SBOM：路径、hash、来源 URL/包版本、版权方、SPDX、是否修改。
2. 对 HNU/命名作者/无头文件取得书面授权，或用许可清晰的实现替换。
3. 把 Apache、CERN-OHL-P、MIT、CMSIS、ST HAL/device、SLA0044、SEGGER 条款和 NOTICE 一并放入发布包。
4. 固件二进制建立对应 commit、工具链版本、链接 map、依赖许可和可复现 hash。
5. 单独审查 PDF、预编译库、CAD 和商标；不因“仓库已经公开”推定许可。
6. 将 `python tools/check_license_policy.py --profile repository-public --enforce` 接入发布流水线；后续再加入 REUSE/ScanCode/FOSSology 类扫描并人工复核文件级例外。扫描工具不能替代授权证据。

资料收集要求见
[17-materials-supplement-checklist.md](17-materials-supplement-checklist.md)；
许可证结论、发布门禁和签署步骤见
[16-license-verification-checklist.md](16-license-verification-checklist.md)。

## 6. 审计结论

仓库已有较谨慎的多许可证顶层结构，并已补充可机读组件清单和发布阻断检查；FreeRTOS、CMSIS、HAL、ST USB 和 SEGGER 等主要组件保留了可识别声明。【较可信｜B级】 但治理工具不能创造许可：活动控制代码中仍有 `all rights reserved` 与无明确来源/许可文件，现阶段不能把整个固件作为 Apache-2.0 作品重新发布，也不能确认所有源代码和固件二进制均可对外再分发。发布前必须解除检查器列出的阻断项，并完成逐文件 SBOM、权利链和厂商条款复核。
