# Engineering Robot 固件分析文档

> 当前阶段：第四阶段——安全、代码质量、许可证与最终验收（静态审查完成）  
> 最终复核日期：2026-07-23  
> 分析版本：`d7aa4214765a36a291678e58afd736484ac28083`（审查开始时的 Git HEAD）  
> 分析对象：`Firmware/Engineer_arm/`、`Firmware/Engineer_chassis/`

## 文档目标与边界

本目录用于建立一套有源码依据、可追溯、适合操作手、嵌入式初学者和固件维护开发者使用的中文固件技术文档。

第一阶段回答“代码在哪里”；第二阶段建立端到端控制证据链；第三阶段把证据转化为教学和维护材料；第四阶段重新检查关键源码，完成安全、并发、代码质量、许可证、证据和交叉一致性验收。本文档集可用于代码学习和静态维护，但仍不是经实机验证的正式操作手册。四个阶段均未修改固件控制逻辑或参数。

关键结论使用以下可信度：

- `【已确认｜A级】`：调用、赋值、协议或构建链可由代码直接闭合。
- `【较可信｜B级】`：多处代码相互支持，但仍缺硬件或运行验证。
- `【推测｜C级】`：仅有局部代码、命名或注释支持。
- `【代码中未确认｜D级】`：仓库缺少足够证据。

## 当前进度

| 项目 | 状态 | 结果摘要 |
| --- | --- | --- |
| 两个固件工程定位 | 已完成 | 实际目录为 `Firmware/Engineer_arm/` 与 `Firmware/Engineer_chassis/`，仓库目录名为大写 `Firmware` |
| 构建入口与启动入口 | 已完成 | 两个工程均由 Makefile 编译顶层 `Src/`，不是同目录下重复的 `Core/Src/` |
| STM32 外设扫描 | 已完成 | 两工程 `.ioc` 配置相同的 CAN、UART、SPI、I²C、TIM、ADC、DAC、RTC、CRC、RNG、USB 等外设集合 |
| FreeRTOS 任务索引 | 已完成 | 每个固件镜像可确认创建 6 个活动任务；另有被注释或未调用的任务创建代码 |
| 中断与回调索引 | 已完成 | 每工程 `Src/stm32f4xx_it.c` 定义 35 个异常/中断处理函数；BSP 与 USB 层另有 22 个 HAL 回调 |
| 设备、算法与输入入口 | 已完成 | 已区分当前初始化、仅有驱动、以及接收已初始化但未进入活动控制分支的模块 |
| 代码来源与许可证 | 已完成（初扫） | 已区分项目代码、CubeMX/ST 代码、HAL/CMSIS/FreeRTOS/SEGGER 等第三方来源；不构成法律结论 |
| 操作输入与模式 | 已完成（静态） | 遥控器、图传键鼠、自定义控制器、软件 STOP 和自动动作状态已建立映射 |
| 机械臂控制链 | 已完成（静态） | 6 路位置通道 + 1 路速度通道、2 路 PWM 舵机；目标—控制器—CAN—反馈闭环已定位 |
| 底盘控制链 | 已完成（静态） | 四轮速度解算、两履带、五姿态电机、IMU 与功率缩放链已定位 |
| 板间通信 | 已完成（静态） | Arm CAN1 `0x311` → Chassis CAN2；反向 `0x312`；字段、周期和超时均已整理 |
| 安全行为 | 已完成初步审计 | 识别板间失联保留旧命令、FSM 表越界等高风险；未修改源码、未做实机验证 |
| 启动、RTOS 与中断教学 | 已完成（静态） | 两板复位到任务、12 个活动任务实例、ISR/回调生产者—消费者关系已展开 |
| 控制算法与数据字典 | 已完成（静态） | 31 个活动 PID 环、轨迹、底盘运动学、INS EKF、滤波和功率模型已解释 |
| 参数与构建调试 | 已完成（环境验证） | 44 项关键运行参数；两工程 `make -n all` 成功，当前环境缺 `arm-none-eabi-gcc`，未实际编译或烧录 |
| 驱动、故障排查与学习路线 | 已完成（静态） | 四类活动电机、BMI088、遥控器/图传与 18 类常见故障形成教学索引 |
| 安全与并发专项审查 | 已完成（静态） | 完成安全机制矩阵、实时性/共享数据审查和 14 项待确认信息；确认 5 个 P0 |
| 代码质量与证据复核 | 已完成（静态） | 复核操作映射、模式、双板控制链、协议、31 个 PID、12 个任务和关键参数；问题总计 P0=5/P1=7/P2=11/P3=6 |
| 许可证最终审计 | 已完成（含发布治理门禁） | 确认混合许可结构；已建立 15 类组件清单和只读发布检查；HNU ARR、无明确许可算法/驱动、ST USB SLA0044 仍是外部授权阻断 |
| 统一未确认信息清单 | 已建立 | 汇总硬件/电气、操作、控制、构建和许可证 40 项待核验信息，并给出材料、方法、风险和负责人 |
| 文档最终验收 | 有条件通过 | 可用于学习和维护；操作内容仍需硬件资料、P0 修复和实机安全验证 |

## 两个固件工程概览

### 机械臂工程

`【已确认｜A级】` `Firmware/Engineer_arm/` 是当前机械臂固件工程。Makefile 编译 `Src/main.c`；该入口初始化外设后调用 `RobotInit()`，而 `RobotInit()` 启用 `RobotCMDInit()`、`GimbalInit()` 和 `OSTaskInit()`。

依据：

- `Firmware/Engineer_arm/Makefile:39-57`：源文件清单以 `Src/main.c` 开始。
- `Firmware/Engineer_arm/Src/main.c:80,97,104-136`：`main()`、外设初始化、`RobotInit()`、`MX_FREERTOS_Init()` 与 `osKernelStart()`。
- `Firmware/Engineer_arm/application/robot.c:18-32`：`RobotInit()` 的活动初始化调用。

本阶段确认的活动业务侧重点是机械臂（代码中沿用 `gimbal` 命名）、机械臂命令处理、图传键鼠、遥控器、USB 上位机接收、裁判系统/UI 与主从板 CAN 通信。USB 上位机数据虽被接收，但其控制处理调用当前被注释，不能视为活动控制入口。

### 底盘工程

`【已确认｜A级】` `Firmware/Engineer_chassis/` 是当前底盘固件工程。Makefile 同样编译 `Src/main.c`；`RobotInit()` 启用 `RobotCMDInit()`、`ChassisInit()` 和 `OSTaskInit()`，而 `GimbalInit()` 被注释。

依据：

- `Firmware/Engineer_chassis/Makefile:39-57`：源文件清单以 `Src/main.c` 开始。
- `Firmware/Engineer_chassis/Src/main.c:80,97,104-136`：实际入口和调度器启动。
- `Firmware/Engineer_chassis/application/robot.c:18-32`：活动初始化为 `RobotCMDInit()` 与 `ChassisInit()`。

底盘的活动命令路径从板间 CAN 接收 `Master_Cmd_Data_s`，再写入 `chassis_cmd_send`；本地遥控器接收仍被初始化，但 `RobotCMDTask()` 中本地遥控/键鼠分支整体被注释。因此，“底盘当前由机械臂板经 CAN 下发命令”是代码直接支持的结论，而不是根据目录名推测。

依据：

- `Firmware/Engineer_chassis/application/cmd/robot_cmd.c:103,121-132`：遥控器和板间 CAN 实例初始化。
- `Firmware/Engineer_chassis/application/cmd/robot_cmd.c:562-572,606-619`：活动 CAN 数据复制，以及本地输入分支被注释。
- `Firmware/Engineer_arm/application/cmd/robot_cmd.c:121-132,939-941`：互补的 CAN ID 与命令发送。

## 按读者选择入口

- **操作手：** 先读 [04-operator-guide.md](04-operator-guide.md)，再读 [11-safety-analysis.md](11-safety-analysis.md) 和 [12-troubleshooting.md](12-troubleshooting.md)。所有动作仅是代码映射，不是正式上机指令。
- **STM32/FreeRTOS 初学者：** 按 [13-beginner-learning-path.md](13-beginner-learning-path.md) → [01-repository-map.md](01-repository-map.md) → [03-startup-and-rtos.md](03-startup-and-rtos.md) → [09-hardware-drivers.md](09-hardware-drivers.md) 阅读。
- **控制学习者：** 读 [02-system-architecture.md](02-system-architecture.md) → [05-arm-control.md](05-arm-control.md) / [06-chassis-control.md](06-chassis-control.md) → [08-control-algorithms.md](08-control-algorithms.md)。
- **固件开发者/维护者：** 从 [14-code-quality-review.md](14-code-quality-review.md) 的 P0/P1 开始，再读协议、参数和故障排查。
- **发布/合规负责人：** 先读 [15-license-audit.md](15-license-audit.md)，按 [17-materials-supplement-checklist.md](17-materials-supplement-checklist.md) 收集来源和版本资料，再按 [16-license-verification-checklist.md](16-license-verification-checklist.md) 完成许可结论与发布审批，并结合仓库根 `LICENSE`、`NOTICE`、`Firmware/LICENSE` 执行。
- **项目负责人/验收负责人：** 使用 [18-unconfirmed-information-checklist.md](18-unconfirmed-information-checklist.md) 分派 40 项待确认工作并记录闭环证据。

## 完整文档目录与推荐顺序

1. [01-repository-map.md](01-repository-map.md)：第一阶段完整仓库地图、入口、构建、任务、中断、设备、算法、输入、许可证和后续阅读清单。
2. [02-system-architecture.md](02-system-architecture.md)：双板职责、任务/中断关系、模式状态、总数据流与安全摘要。
3. [03-startup-and-rtos.md](03-startup-and-rtos.md)：两板从复位到任务、任务时间轴、中断/DMA 与共享数据风险。
4. [04-operator-guide.md](04-operator-guide.md)：基于代码的输入映射与模式草案；不是上机操作指令。
5. [05-arm-control.md](05-arm-control.md)：机械臂 7 路电机、轨迹、闭环、反馈与停止行为。
6. [06-chassis-control.md](06-chassis-control.md)：四轮解算、姿态电机、功率限制、IMU 与安全行为。
7. [07-communication-protocols.md](07-communication-protocols.md)：板间、遥控器、图传、USB 和电机 CAN 协议。
8. [08-control-algorithms.md](08-control-algorithms.md)：31 个活动 PID、轨迹、运动学、滤波、EKF、功率模型和数据字典。
9. [09-hardware-drivers.md](09-hardware-drivers.md)：BSP/设备分层、活动电机、BMI088、遥控器和其他驱动。
10. [10-parameters-and-tuning.md](10-parameters-and-tuning.md)：44 项关键参数、安全调试、编译、烧录、GDB/RTT/CAN 调试。
11. [11-safety-analysis.md](11-safety-analysis.md)：完整安全机制矩阵、实时性/并发风险、待确认事项和实机前门槛。
12. [12-troubleshooting.md](12-troubleshooting.md)：低风险优先的故障树、18 类现象和代码级高风险。
13. [13-beginner-learning-path.md](13-beginner-learning-path.md)：16 阶段学习路线和 8 条深入代码路径。
14. [14-code-quality-review.md](14-code-quality-review.md)：29 项 P0–P3 问题、证据复核、重大修正和改进路线。
15. [15-license-audit.md](15-license-audit.md)：第三方分类、许可边界、再分发风险和发布前清单。
16. [16-license-verification-checklist.md](16-license-verification-checklist.md)：许可证、权利人授权、条款结论、发布门禁和签署代办。
17. [17-materials-supplement-checklist.md](17-materials-supplement-checklist.md)：来源文件、版本、hash、SBOM、CAD/PDF 和构建记录补充代办。
18. [18-unconfirmed-information-checklist.md](18-unconfirmed-information-checklist.md)：全项目 40 项未确认信息、所需材料、核验方法、风险、负责人和关闭模板。
19. [glossary.md](glossary.md)：当前可由仓库确认的术语、代码名称和谨慎释义。

## 当前已经确认的信息

- `【已确认｜A级】` 两工程芯片均为 STM32F407IGH6；依据为两个 `Engineer.ioc:141,171`。
- `【已确认｜A级】` 两工程以 GNU Make + `arm-none-eabi-*` 工具链构建，链接脚本为 `STM32F407IGHx_FLASH.ld`，目标为 `build/ENGINEER.elf/.hex/.bin`；依据为两个 `Makefile:17,182-194,308,318-344`（机械臂工程对应行号整体早 1 行）。
- `【已确认｜A级】` 当前 Makefile 编译顶层 `Src/`、`Inc/`、`application/`、`bsp/`、`modules/`、`Drivers/` 和 `Middlewares/`；同工程的 `Core/` 与 `USB_DEVICE/` 不在当前源文件清单中。
- `【已确认｜A级】` 两工程均使用 CMSIS-RTOS v1 风格的 `osThreadDef`/`osThreadCreate`，时基配置为 1 kHz；依据为 `application/robot_task.h`、`Src/freertos.c` 与 `Inc/FreeRTOSConfig.h:59-70`。
- `【已确认｜A级】` 板间链路实际是机械臂 `CAN1` 发送 `0x311`/接收 `0x312`，底盘 `CAN2` 发送 `0x312`/接收 `0x311`。依据为两个 `application/cmd/robot_cmd.c:121-132`。第一阶段只写 ID、未明确两端使用不同 CAN 外设，现已补正。
- `【已确认｜A级】` 机械臂当前为 7 路电机控制：前 6 路位置目标，第 7 路 LK4005 为速度目标；依据为 `Engineer_arm/application/gimbal/gimbal.c:27-287,352-397`。
- `【已确认｜A级】` 底盘当前控制 11 个电机：4 驱动轮、2 履带、2 DM 摇臂、2 RS 后腿和 1 RS 姿态/方向电机；依据为 `Engineer_chassis/application/chassis/chassis.c:54-253`。
- `【已确认｜A级】` 板间 CAN 超时不会清空旧命令，且底盘活动任务没有使用 `CANCommIsOnline()`；这是当前最重要的静态安全风险。依据为 `can_comm.c:73-78,126-135` 与底盘 `robot_cmd.c:562-624`。
- `【已确认｜A级】` 两工程当前没有发现应用层 RTOS 队列、信号量、互斥锁、事件组、任务通知或软件定时器的创建/使用；`bsp_tools.c` 存在基于 `osSignalWait` 的回调任务工具，但没有找到调用者。
- `【已确认｜A级】` IWDG/WWDG HAL 模块宏均被注释，`.ioc` 中也没有看门狗配置；依据为两个 `Inc/stm32f4xx_hal_conf.h:58,72` 及 `.ioc` 搜索结果。
- `【已确认｜A级】` 活动闭环共 31 个 PID：机械臂 13 个、底盘 18 个；统一 PID 使用 DWT 实测 `dt`，但积分限幅函数含跨实例共享的 `static` 临时量。依据为两工程 `modules/algorithm/controller.c` 及应用电机配置。
- `【已确认｜A级】` 所有周期任务使用相对 `osDelay()`，未使用 `vTaskDelayUntil()`；名义 1/5/10 ms 不能等同于严格周期。依据为两工程 `application/robot_task.h` 与 `FreeRTOSConfig.h`。
- `【已确认｜A级】` 当前环境能对两工程成功执行 `make -n all`，但 `arm-none-eabi-gcc` 不在 PATH，故本阶段没有实际编译或烧录。
- `【已确认｜A级】` 最终审查问题数量为 P0=5、P1=7、P2=11、P3=6；编号和逐项证据见 `14-code-quality-review.md`。
- `【已确认｜A级】` 软件 STOP 依赖 MCU 任务和通信，不等同于独立硬件急停；板间超时、Fault 和底盘预初始化路径均不能证明所有输出被覆盖。

## 当前未确认的信息

完整清单、材料要求和关闭模板见
[18-unconfirmed-information-checklist.md](18-unconfirmed-information-checklist.md)。当前最关键的未确认信息包括：

- `【代码中未确认｜D级】` 两块控制板的具体硬件名称、PCB 版本、物理安装位置和接线对应关系。
- `【代码中未确认｜D级】` 电机在真实机械结构中的正方向、关节编号与装配图之间的一一对应。
- `【代码中未确认｜D级】` 遥控器准确型号、面板标签、开关物理方向与操作手习惯；代码可确认 SA—SD、J1—J4 数值条件，但不足以生成经安全验证的正式操作手册。
- `【代码中未确认｜D级】` 当前 `build/` 产物由哪个源码提交、工具链版本和参数生成；目录中有产物，但没有可验证的构建记录。
- `【代码中未确认｜D级】` 固件能否在实机安全启动。第一阶段没有烧录、连接硬件或实机验证。
- `【代码中未确认｜D级】` 独立硬件急停、电机电源接触器或驱动 enable 回路是否存在并覆盖 MCU/CAN 故障。
- `【代码中未确认｜D级】` HNU YueLu EC ARR 文件、命名作者算法、部分电机/传感器驱动和厂商资料的完整授权链。
- `【较可信｜B级】` 两工程大部分基础代码同源复制，但局部业务代码、文件名和活动分支不同；需要后续以语义差异而非目录整体相同来维护文档。

## 最终验收与后续工作

静态文档验收为**有条件通过**：代码地图、双板控制链、任务/中断、协议、算法、参数、故障、安全和许可证入口可以支持学习及维护；但 P0 风险尚未修复，也没有机械、电气和实机证据，操作手内容不能作为正式操作依据。

后续不属于本次文档阶段的工作应按 [14-code-quality-review.md](14-code-quality-review.md) 的路线执行：先处理 P0/P1，再完成无动力单元测试、独立硬件急停验证、逐电机低功率台架、板间/传感器故障注入和 HIL；同时以根目录 `THIRD_PARTY_COMPONENTS.json` 为组件级基线补齐逐文件 SBOM 与授权链，并在发布前运行 `python tools/check_license_policy.py --profile repository-public --enforce`。只有验证结果可追溯到固件、机械和电气版本后，才能升级操作手手册。

## 安全声明

本目录当前只描述静态代码事实。它不证明机器人可安全运行，不替代机械装配图、电气图、遥控器说明、限位/急停设计和实机验证。
