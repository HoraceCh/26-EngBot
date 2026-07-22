# 26-EngBot

RoboMaster 2026 工程机器人研发项目，包含机械臂与底盘固件、整机机械设计、加工图和物料清单。

> 本仓库保存的是研发中的工程资料，不代表可直接投入比赛或安全运行的完整产品。烧录、上电和机构调试前，请完成限位、急停、绝缘、过流保护与人员安全检查。

## 项目组成

| 部分 | 内容 | 主要工具 |
| --- | --- | --- |
| `Firmware/2026-engineer/Engineer_arm` | 机械臂控制固件 | STM32CubeMX、GNU Arm Embedded Toolchain、Make、OpenOCD/J-Link |
| `Firmware/2026-engineer/Engineer_chassis` | 底盘控制固件 | STM32CubeMX、GNU Arm Embedded Toolchain、Make、OpenOCD/J-Link |
| `Mechanical/CAD` | 整机、云台、底盘、车架和裁判系统的 SolidWorks 模型 | SolidWorks |
| `Mechanical/Dxf` | 可交付加工的板件展开图 | 支持 DXF 的 CAD/CAM 软件 |
| `Mechanical/BOM` | 电机等物料清单 | Excel 或兼容软件 |

两套固件工程均以 STM32F407I(E-G)Hx（UFBGA176）为目标，使用 12 MHz 外部晶振、FreeRTOS、STM32 HAL/CMSIS 和 Makefile 构建。它们是彼此独立的工程，生成物名称均为 `ENGINEER`，请在各自目录中编译。

## 固件目录约定

每套固件大致采用以下分层：

- `application/`：机器人任务、命令处理和上层控制逻辑。
- `bsp/`：CAN、UART、USB、SPI、I²C、PWM、Flash 等板级接口。
- `modules/`：电机、IMU、遥控器、裁判系统、消息中心、算法等通用模块。
- `Core/`、`Drivers/`、`Middlewares/`、`USB_DEVICE/`：STM32CubeMX 生成代码及厂商/第三方组件。
- `Engineer.ioc`：STM32CubeMX 工程配置。
- `Makefile`：GNU Arm 工具链构建入口。

修改 CubeMX 管理的文件时应保留 `USER CODE` 区域；重新生成代码前建议先提交当前修改并检查生成差异。

## 开发环境

必需工具：

- Git 与 [Git LFS](https://git-lfs.com/)；CAD、PDF、表格和预编译库通过 LFS 管理。
- GNU Arm Embedded Toolchain，命令前缀为 `arm-none-eabi-`。
- GNU Make。

按需安装：

- STM32CubeMX：修改外设和时钟配置。
- OpenOCD + CMSIS-DAP，或 SEGGER J-Link/J-Flash：下载和调试。
- VS Code + Cortex-Debug + Makefile Tools：使用仓库内共享任务和调试配置。
- SolidWorks：编辑原生机械模型。

首次克隆：

```bash
git lfs install
git clone git@github.com:HoraceCh/26-EngBot.git
cd 26-EngBot
git lfs pull
```

VS Code 的本机工具链路径保存在各工程的 `.vscode/settings.json`，该文件已被 Git 忽略。请按本机安装位置配置 `armToolchainPath`、`openocdPath`、`JLinkGDBServerPath` 和 `gdbPath`。

## 编译固件

机械臂：

```bash
cd Firmware/2026-engineer/Engineer_arm
make -j
```

底盘：

```bash
cd Firmware/2026-engineer/Engineer_chassis
make -j
```

成功后会在当前工程的 `build/` 中生成：

- `ENGINEER.elf`：调试符号和可执行映像。
- `ENGINEER.hex`：Intel HEX 固件。
- `ENGINEER.bin`：裸二进制固件。

清理工程可运行 `make clean`。当前 `clean` 规则使用 Windows `rd` 命令；在 Linux/macOS 下需要自行删除 `build/` 或调整该规则。

## 下载与调试

连接 CMSIS-DAP：

```bash
make download_dap
```

连接 J-Link/J-Flash：

```bash
make download_jlink
```

下载前请确认目标板型号、SWD 接线、供电电压和当前所在工程。VS Code 的 `launch.json` 还提供 DAP/J-Link 的启动与附加调试配置。

## 机械资料

- `Mechanical/CAD` 保存 SolidWorks 零件与装配体，包含部分厂商/供应商模型。
- `Mechanical/Dxf` 保存按材料厚度和数量命名的加工文件。
- `Mechanical/BOM` 保存物料表。

机械文件体积较大，必须通过 Git LFS 获取。提交前请关闭 SolidWorks，确认没有 `~$` 开头的锁文件，并避免把自动恢复文件或重复导出文件加入仓库。

## 协作建议

1. 从 `main` 创建短生命周期分支。
2. 固件修改至少完成对应工程编译；机械修改同时更新受影响的装配体、加工图和 BOM。
3. 不提交 `build/`、个人调试状态、本机 IDE 路径或 CAD 锁文件。
4. Pull Request 中说明变更目标、测试方法、硬件风险和回退方式。
5. 厂商代码或第三方模型必须保留原版权与许可证声明。

## 许可证

本仓库采用按内容分区的多许可证结构：

- 团队原创固件贡献：`Apache-2.0`，范围见 [`Firmware/LICENSE`](Firmware/LICENSE)。
- 团队原创机械设计：`CERN-OHL-P-2.0`，范围见 [`Mechanical/LICENSE`](Mechanical/LICENSE)。
- STM32、CMSIS、FreeRTOS、SEGGER、HNU YueLu EC、厂商模型、数据手册和协议资料等第三方内容仍适用其各自声明，不由上述许可证重新授权。

完整文本和许可边界见根目录 [`LICENSE`](LICENSE)、[`LICENSES/`](LICENSES) 与 [`NOTICE`](NOTICE)。如文件自身的版权或许可证声明与目录说明不同，以文件自身声明为准。
