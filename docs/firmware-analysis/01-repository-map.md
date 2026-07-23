# 第一阶段：仓库地图与代码入口索引

## 1. 扫描口径

本页记录静态扫描结果，不完整解释机器人控制逻辑。扫描优先级为：

1. 先以 Makefile 的 `C_SOURCES`、`ASM_SOURCES` 确定实际编译单元；
2. 再检查 `.ioc`、启动文件、`Src/`、`Inc/`、`application/`、`bsp/`、`modules/`；
3. `Core/`、`USB_DEVICE/` 用于识别重复生成内容，但不把未进入 Makefile 的文件当作当前运行入口；
4. `Drivers/`、`Middlewares/`、`build/` 用于识别依赖、许可证和构建方式，不逐文件解释；
5. 代码注释只作辅助证据，活动调用、赋值和构建清单优先。

路径均相对仓库根目录。两个工程存在大量同名文件，表中只写 `Engineer_*` 时表示两个工程各有一份独立副本。

## 2. 仓库与工程概览

### 2.1 顶层结构

```text
26-EngBot/
├── Firmware/
│   ├── LICENSE
│   ├── Engineer_arm/
│   │   ├── application/{cmd,gimbal,chassis,shoot}/
│   │   ├── bsp/{can,usart,usb,spi,iic,gpio,pwm,tim,dwt,...}/
│   │   ├── modules/{motor,remote,referee,imu,algorithm,can_comm,...}/
│   │   ├── Src/  Inc/                 # 当前 Makefile 使用
│   │   ├── Core/  USB_DEVICE/         # 重复生成树，当前 Makefile 未使用
│   │   ├── Drivers/  Middlewares/     # HAL/CMSIS/FreeRTOS/USB/SEGGER 等
│   │   ├── build/                     # 现有编译产物
│   │   ├── Engineer.ioc  Makefile
│   │   └── startup_stm32f407xx.s  STM32F407IGHx_FLASH.ld
│   └── Engineer_chassis/              # 结构同上，业务活动分支不同
├── Mechanical/
├── LICENSES/
├── LICENSE
├── NOTICE
└── README.md
```

### 2.2 目录分类表

| 路径 | 所属工程 | 分类 | 主要作用 | 是否重点分析 | 证据等级 |
| --- | --- | --- | --- | --- | --- |
| `Firmware/Engineer_arm/` | 机械臂 | 固件工程根 | 机械臂固件、构建配置及依赖 | 是 | 【已确认｜A级】 |
| `Firmware/Engineer_chassis/` | 底盘 | 固件工程根 | 底盘固件、构建配置及依赖 | 是 | 【已确认｜A级】 |
| `*/startup_stm32f407xx.s` | 两者 | 启动和芯片初始化 | 向量表、`Reset_Handler`、调用 `SystemInit` 和 `main` | 是 | 【已确认｜A级】；`startup...s:57-100` |
| `*/Src/`、`*/Inc/` | 两者 | CubeMX 生成框架 + 项目 USER CODE | 当前 Makefile 使用的入口、外设初始化、中断、USB 和 RTOS 框架 | 是 | 【已确认｜A级】；`Makefile:39-57` |
| `*/Core/` | 两者 | 重复的 CubeMX 生成树 | 与顶层 `Src/Inc` 并存，但当前 Makefile 未列入 | 低；仅作差异风险检查 | 【已确认｜A级】；Makefile 无 `Core/` 源文件 |
| `*/USB_DEVICE/` | 两者 | 重复的 CubeMX USB 树 | USB Device App/Target 结构；当前 Makefile 使用的是顶层 `Src/usbd_*.c` | 低 | 【已确认｜A级】；`Makefile:52-55` |
| `*/application/` | 两者 | 业务逻辑 | `robot`、命令、机械臂、底盘、发射模块 | 是 | 【已确认｜A级】；`Makefile:165-169` 附近 |
| `*/application/cmd/` | 两者 | 控制入口与业务编排 | 输入解析结果到控制命令/状态的首次业务写入、板间通信 | 最高 | 【已确认｜A级】；`RobotCMDInit/RobotCMDTask` |
| `Engineer_arm/application/gimbal/` | 机械臂 | 机械臂业务逻辑 | 初始化并更新 RS、DM、LK 电机和两个 PWM 舵机 | 最高 | 【已确认｜A级】；`gimbal.c:27-310` |
| `Engineer_chassis/application/chassis/` | 底盘 | 底盘业务逻辑 | 初始化 IMU、DJI/RS/DM 电机、底盘 PID 与功率控制 | 最高 | 【已确认｜A级】；`chassis.c:56-274` |
| 另一工程中未启用的 `application/gimbal`/`chassis` | 各自 | 编译但未由 `RobotInit()` 启用的业务代码 | Makefile 仍编译文件，初始化调用被注释 | 中；防止误判 | 【已确认｜A级】；两个 `application/robot.c:27-29` |
| `*/bsp/` | 两者 | 板级支持代码 | 对 HAL 的 CAN、UART、SPI、I²C、GPIO、PWM、USB、DWT、日志等封装 | 是 | 【已确认｜A级】；`bsp/bsp_init.h:16-20` 与各注册函数 |
| `*/modules/motor/` | 两者 | 设备驱动 + 控制 | 多种电机协议、反馈解析、PID 和发送 | 是 | 【已确认｜A级】 |
| `*/modules/remote/` | 两者 | 输入协议 | SBUS/WBUS 遥控器解析、UART 注册、离线守护 | 是 | 【已确认｜A级】；`remote_control.c:35-162` |
| `*/modules/referee/` | 两者 | 通信协议 + UI | 裁判系统、图传键鼠/自定义数据、UI | 是 | 【已确认｜A级】 |
| `*/modules/can_comm/` | 两者 | 板间通信协议 | 多帧 CAN 封包、CRC8、超时守护 | 是 | 【已确认｜A级】 |
| `*/modules/message_center/` | 两者 | 业务通信基础设施 | 内存发布/订阅 | 是 | 【已确认｜A级】 |
| `*/modules/algorithm/` | 两者 | 控制算法 | PID、Kalman/EKF、运动学、轨迹、CRC、限幅工具 | 是 | 【已确认｜A级】 |
| `*/modules/imu/`、`*/modules/BMI088/` | 两者 | 设备驱动 + 状态估计 | BMI088 访问和 INS 姿态解算 | 底盘为重点，机械臂次要 | 【已确认｜A级】；底盘 `ChassisInit()` 调用 `INS_Init()` |
| `*/modules/daemon/` | 两者 | 安全和故障处理基础设施 | 软件在线计数、超时回调 | 是 | 【已确认｜A级】；`daemon.c:42-57` |
| `*/modules/{bluetooth,encoder,oled,TFminiPlus,...}` | 两者 | 设备驱动库存 | 驱动已存在，不等于活动业务已初始化 | 中/低 | 【已确认｜A级】（存在性）；活动性逐项另标 |
| `Engineer_chassis/modules/power_control/` | 底盘 | 控制算法 | 底盘功率预测与速度目标缩放 | 是 | 【已确认｜A级】；`chassis.c:267-321` |
| `*/Drivers/` | 两者 | HAL/CMSIS 第三方代码 | ST HAL、Arm/ST CMSIS | 低；仅接口与许可证 | 【已确认｜A级】 |
| `*/Middlewares/` | 两者 | 第三方代码 | FreeRTOS、CMSIS-RTOS、ST USB、CMSIS-DSP、SEGGER RTT | 低；仅接口与许可证 | 【已确认｜A级】 |
| `*/build/` | 两者 | 编译产物 | `.o/.d/.lst/.elf/.hex/.bin/.map` | 否 | 【已确认｜A级】 |
| `.vscode/`、`*.jdebug*`、`*.jflash`、`openocd_*.cfg` | 两者 | 调试/烧录配置 | VS Code、J-Link/J-Flash、OpenOCD 配置 | 构建维护阶段分析 | 【已确认｜A级】 |
| `Mechanical/` | 非固件 | 机械设计 | 不属于本阶段固件源码扫描 | 否 | 【已确认｜A级】 |
| `docs/firmware-analysis/` | 文档 | 分析文档 | 本任务输出 | 是 | 【已确认｜A级】 |

## 3. 实际入口与启动索引

### 3.1 Makefile 决定的入口

`【已确认｜A级】` 两工程当前实际编译入口是顶层 `Src/main.c`，而不是 `Core/Src/main.c`。

依据：

- 两个 Makefile 的 `C_SOURCES` 从 `Src/main.c` 开始，并列出顶层 `Src/*.c`；机械臂 `Makefile:39-57`，底盘相同行段。
- `Core/Src/main.c` 的 USER CODE 区没有 `RobotInit()`；顶层 `Src/main.c:127` 明确调用 `RobotInit()`。
- 两个 Makefile 均未出现 `Core/` 或 `USB_DEVICE/` 源路径。

这意味着后续分析若只沿 CubeMX 默认目录 `Core/` 阅读，会得到“只有默认任务、不启动业务”的错误结论。

### 3.2 启动流程索引

| 工程 | 启动文件 | `main()` | RTOS 初始化 | 调度器启动 | 业务初始化 | 主要依据 |
| --- | --- | --- | --- | --- | --- | --- |
| 机械臂 | `Engineer_arm/startup_stm32f407xx.s` | `Engineer_arm/Src/main.c:80` | `MX_FREERTOS_Init()`，`:133` | `osKernelStart()`，`:136` | `RobotInit()`，`:127` → `RobotCMDInit/GimbalInit/OSTaskInit` | 启动文件 `:57-100`; `application/robot.c:18-32` |
| 底盘 | `Engineer_chassis/startup_stm32f407xx.s` | `Engineer_chassis/Src/main.c:80` | `MX_FREERTOS_Init()`，`:133` | `osKernelStart()`，`:136` | `RobotInit()`，`:127` → `RobotCMDInit/ChassisInit/OSTaskInit` | 启动文件 `:57-100`; `application/robot.c:18-32` |

两工程共同的入口顺序索引为：

```text
Reset_Handler
→ SystemInit
→ __libc_init_array
→ main
→ HAL_Init
→ SystemClock_Config
→ MX_GPIO/DMA/UART/CAN/... 初始化
→ RobotInit
→ BSPInit + RobotCMDInit + GimbalInit 或 ChassisInit + OSTaskInit
→ MX_FREERTOS_Init（创建 defaultTask）
→ osKernelStart
```

`【已确认｜A级】` `RobotInit()` 在调度器启动前创建业务任务；随后 `MX_FREERTOS_Init()` 再创建 CubeMX `defaultTask`。依据为 `Src/main.c:127-136`、`application/robot.c:18-32`、`Src/freertos.c:83-107`。

## 4. 构建方式

### 4.1 构建配置表

| 工程 | 芯片 | 构建系统 | 工具链 | 链接脚本 | 输出文件 | 当前能否构建 | 可信度 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 机械臂 | STM32F407IGH6 / Cortex-M4F | GNU Make；CubeMX 目标工具链为 Makefile | `arm-none-eabi-gcc/objcopy/size` | `STM32F407IGHx_FLASH.ld` | `build/ENGINEER.elf/.hex/.bin`，另有 `.map` | 未实际编译；`make -n all` 成功展开，但本机 PATH 缺 `arm-none-eabi-gcc` | 【已确认｜A级】 |
| 底盘 | STM32F407IGH6 / Cortex-M4F | GNU Make；CubeMX 目标工具链为 Makefile | `arm-none-eabi-gcc/objcopy/size` | `STM32F407IGHx_FLASH.ld` | 同上 | 未实际编译；`make -n all` 成功展开，但本机 PATH 缺交叉编译器 | 【已确认｜A级】 |

证据：

- 芯片：两个 `Engineer.ioc:141,171`。
- 工具链：机械臂 `Makefile:181-193`，底盘 `Makefile:182-194`。
- CPU/FPU：机械臂 `Makefile:202-211`，底盘 `Makefile:203-212`。
- 链接与输出：机械臂 `Makefile:307-343`，底盘 `Makefile:308-344`。
- CubeMX 工具链选择：两个 `Engineer.ioc:481` 为 `Makefile`。
- 只读探测：GNU Make 4.2.1 可用；`arm-none-eabi-gcc --version` 返回“not found”；没有执行实际编译、清理、下载或烧录。

### 4.2 调试与烧录配置

| 配置 | 两工程情况 | 证据 | 注意 |
| --- | --- | --- | --- |
| OpenOCD | `openocd_dap.cfg`、`openocd_jlink.cfg` 存在 | 工程根目录；Makefile `download_dap` | 本阶段未执行 |
| J-Link/J-Flash | `*.jdebug*`、`stm32.jflash` 存在 | 工程根目录；Makefile `download_jlink` | 本阶段未执行 |
| Keil | 未找到 `*.uvprojx` | 仓库文件扫描 | 【代码中未确认｜D级】是否有仓库外工程文件 |
| STM32CubeIDE | 未找到 `.project/.cproject` | 仓库文件扫描 | `.ioc` 存在，但当前导出目标是 Makefile |
| CMake | 未找到 `CMakeLists.txt` | 仓库文件扫描 | 不应假定支持 CMake |

## 5. STM32 外设索引

`【已确认｜A级】` 两工程 `.ioc` 的外设配置和 `Src/main.c` 初始化调用目前相同。

| 外设 | 实例/配置线索 | 初始化入口 | 主要证据 |
| --- | --- | --- | --- |
| GPIO | 多个普通 GPIO 与 EXTI3/4/9_5 | `MX_GPIO_Init()` | `Src/main.c:104`; `.ioc:259-261` |
| DMA | USART1/3/6、SPI1、I2C2 等 DMA 请求 | `MX_DMA_Init()` | `Src/main.c:105`; `.ioc:30-125` |
| CAN | CAN1、CAN2，均计算为 1 Mbit/s | `MX_CAN1_Init()`、`MX_CAN2_Init()` | `Src/main.c:107-108`; `.ioc:12-29` |
| UART/USART | USART1 921600；USART3 100000、9 位、偶校验、仅接收；USART6 异步 | `MX_USART1/3/6_UART_Init()` | `Src/main.c:106,109,124`; `.ioc:586-597` |
| I²C | I2C2、I2C3，Fast mode | `MX_I2C2_Init()`、`MX_I2C3_Init()` | `Src/main.c:115-116`; `.ioc:136-139` |
| SPI | SPI1 10.5 Mbit/s；SPI2 5.25 Mbit/s、CPOL 高、第二边沿 | `MX_SPI1_Init()`、`MX_SPI2_Init()` | `Src/main.c:117-118`; `.ioc:547-561` |
| Timer/PWM | TIM1/4/5/8/10，TIM6；多个 PWM 通道 | `MX_TIM1/4/5/8/10/6_Init()` | `Src/main.c:119-125`; `.ioc:566-585` |
| ADC | ADC1 内部温度/Vref 配置线索 | `MX_ADC1_Init()` | `Src/main.c:113`; `.ioc:2-8,605-608` |
| DAC | DAC，含 DAC_OUT2 复用线索 | `MX_DAC_Init()` | `Src/main.c:114`; `.ioc:309,521-522` |
| RTC | RTC 启用 | `MX_RTC_Init()` | `Src/main.c:110`; `.ioc:511-515,615-616` |
| CRC | 硬件 CRC 启用 | `MX_CRC_Init()` | `Src/main.c:111`; `.ioc:609-610` |
| RNG | 硬件 RNG 启用 | `MX_RNG_Init()` | `Src/main.c:112`; `.ioc:613-614` |
| USB OTG FS | Device Only，USB CDC | `MX_USB_DEVICE_Init()` 在默认任务入口调用 | `.ioc:598-604`; `Src/freertos.c:122-130` |
| IWDG/WWDG | 未启用 | 无 | `Inc/stm32f4xx_hal_conf.h:58,72` 宏被注释 |

## 6. FreeRTOS 任务清单

### 6.1 活动任务

“活动”表示创建调用未被注释，且其前置条件可由当前初始化链确认。栈大小为传给 CMSIS-RTOS `osThreadDef(..., stacksz)` 的原始数值；本阶段不改写成字节/字以避免跨适配层误读。

| 工程 | 任务名 | 入口函数 | 创建位置 | 优先级 | 栈参数 | 周期线索 | 初步作用 | 可信度 |
| --- | --- | --- | --- | ---: | ---: | --- | --- | --- |
| 机械臂 | `defaultTask` | `StartDefaultTask` | `Src/freertos.c:106-107` | Normal | 128 | `osDelay(1)` | 初始化 USB Device，之后空循环 | 【已确认｜A级】 |
| 机械臂 | `motortask` | `StartMOTORTASK` | `application/robot_task.h:43-44` | Normal | 512 | `osDelay(1)` | 调 `MotorControlTask()`；当前机械臂侧活动 RS/LK 控制 | 【已确认｜A级】 |
| 机械臂 | `daemontask` | `StartDAEMONTASK` | `application/robot_task.h:46-47` | Normal | 128 | `osDelay(10)` | `DaemonTask()` + `BuzzerTask()` | 【已确认｜A级】 |
| 机械臂 | `robottask` | `StartROBOTTASK` | `application/robot_task.h:49-50` | Normal | 1024 | `osDelay(5)` | 调 `RobotTask()`，即命令与机械臂业务任务 | 【已确认｜A级】 |
| 机械臂 | `uitask` | `StartUITASK` | `application/robot_task.h:52-53` | Normal | 1024 | `osDelay(1)`；`RefereeSend()` 另有延时 | UI/裁判系统任务 | 【已确认｜A级】 |
| 机械臂 | `dmmotor` | `DMMotorTask` | `modules/motor/DMmotor/dmmotor.c:366-372` | Normal | 128 | `osDelay(1)`；按实例轮转 | DM 电机控制与 CAN 发送 | 【已确认｜A级】；`GimbalInit()` 先注册 2 个 DM 实例 |
| 底盘 | `defaultTask` | `StartDefaultTask` | `Src/freertos.c:106-107` | Normal | 128 | `osDelay(1)` | 初始化 USB Device，之后空循环 | 【已确认｜A级】 |
| 底盘 | `instask` | `StartINSTASK` | `application/robot_task.h:39-40` | AboveNormal | 1024 | `osDelay(1)` | `INS_Task()` 姿态更新 | 【已确认｜A级】 |
| 底盘 | `motortask` | `StartMOTORTASK` | `application/robot_task.h:43-44` | Normal | 512 | `osDelay(1)` | 调 `MotorControlTask()`；当前底盘侧活动 DJI/RS 控制 | 【已确认｜A级】 |
| 底盘 | `daemontask` | `StartDAEMONTASK` | `application/robot_task.h:46-47` | Normal | 128 | `osDelay(10)` | 软件在线监测与蜂鸣器 | 【已确认｜A级】 |
| 底盘 | `robottask` | `StartROBOTTASK` | `application/robot_task.h:49-50` | Normal | 1024 | `osDelay(5)` | 命令与底盘业务任务 | 【已确认｜A级】 |
| 底盘 | `dmmotor` | `DMMotorTask` | `modules/motor/DMmotor/dmmotor.c:369-375` | Normal | 128 | `osDelay(1)`；按实例轮转 | 两个 DM 摇臂电机控制与 CAN 发送 | 【已确认｜A级】 |

活动任务计数：机械臂 6 个，底盘 6 个，共 12 个固件任务实例。每个固件镜像还会有 FreeRTOS 内核 Idle 任务；是否创建 Timer Service 任务取决于配置，未纳入“项目显式创建任务”计数。

### 6.2 非活动或条件任务

| 工程/模块 | 任务 | 状态 | 证据 |
| --- | --- | --- | --- |
| 机械臂 | `instask` / `StartINSTASK` | 创建代码被注释 | `application/robot_task.h:39-40` |
| 底盘 | `uitask` / `StartUITASK` | 创建代码被注释 | `application/robot_task.h:52-53` |
| 两者 HT 电机 | 每实例 `HTMotorTask` | 仅在调用 `HTMotorControlInit()` 时创建；当前调用被注释 | `modules/motor/HTmotor/HT04.c:218-219`; `robot_task.h:55` |
| 两者 YS 电机 | `YSMotorTask` | `YSMotorControlInit()` 当前被注释 | `modules/motor/YSmotor/ysmotor.c:330-331`; `robot_task.h:56` |
| 两者回调任务工具 | `CallbackTaskBase` | `CreateCallbackTask()` 存在，但未找到调用者 | `bsp/bsp_tools.c:20-57` |

### 6.3 RTOS 同步原语索引

`【已确认｜A级】` 在排除 FreeRTOS/CMSIS-RTOS 第三方实现后，没有找到项目代码创建或使用 Queue、Semaphore、Mutex、Event Group、Task Notification 或 Software Timer。`MX_FREERTOS_Init()` 对应区块为空（`Src/freertos.c:88-103`）。

`bsp/bsp_tools.c:20-57` 实现了基于 `osSignalWait` 的回调任务工具，但全仓没有发现其调用者，因此仅记为“可用基础设施”，不计为活动同步路径。`modules/message_center/` 是自研内存发布/订阅，不是 FreeRTOS Queue。

## 7. 中断与回调清单

### 7.1 异常和 IRQ 入口

两工程 `Src/stm32f4xx_it.c` 的处理函数集合相同，每工程 35 个：6 个 Cortex 异常处理 + 29 个外设 IRQ。表中“两个工程”表示各自独立编译一份。

| 工程 | 类型 | 函数 | 文件/行号 | 关联外设 | 初步处理内容/后续调用 | 可信度 |
| --- | --- | --- | --- | --- | --- | --- |
| 两个工程 | 系统异常 | `NMI_Handler` | `Src/stm32f4xx_it.c:92` | Cortex-M4 | 空循环/默认框架 | 【已确认｜A级】 |
| 两个工程 | 系统故障 | `HardFault_Handler` | `:107` | Cortex-M4 | 无限循环 | 【已确认｜A级】 |
| 两个工程 | 系统故障 | `MemManage_Handler` | `:122` | Cortex-M4 | 无限循环 | 【已确认｜A级】 |
| 两个工程 | 系统故障 | `BusFault_Handler` | `:137` | Cortex-M4 | 无限循环 | 【已确认｜A级】 |
| 两个工程 | 系统故障 | `UsageFault_Handler` | `:152` | Cortex-M4 | 无限循环 | 【已确认｜A级】 |
| 两个工程 | 调试异常 | `DebugMon_Handler` | `:167` | Cortex-M4 | 默认框架 | 【已确认｜A级】 |
| 两个工程 | Flash | `FLASH_IRQHandler` | `:187` | FLASH | `HAL_FLASH_IRQHandler()` | 【已确认｜A级】 |
| 两个工程 | GPIO EXTI | `EXTI3_IRQHandler` | `:201` | `INT_MAG_Pin` | `HAL_GPIO_EXTI_IRQHandler()` → BSP GPIO 回调 | 【已确认｜A级】 |
| 两个工程 | GPIO EXTI | `EXTI4_IRQHandler` | `:215` | `INT_ACC_Pin` | 同上 | 【已确认｜A级】 |
| 两个工程 | GPIO EXTI | `EXTI9_5_IRQHandler` | `:285` | `INT_GYRO_Pin` | 同上 | 【已确认｜A级】 |
| 两个工程 | DMA | `DMA1_Stream1_IRQHandler` | `:229` | USART3 RX | `HAL_DMA_IRQHandler(&hdma_usart3_rx)` | 【已确认｜A级】 |
| 两个工程 | DMA | `DMA1_Stream2_IRQHandler` | `:243` | I2C2 RX | HAL DMA | 【已确认｜A级】 |
| 两个工程 | DMA | `DMA1_Stream7_IRQHandler` | `:398` | I2C2 TX | HAL DMA | 【已确认｜A级】 |
| 两个工程 | DMA | `DMA2_Stream0_IRQHandler` | `:427` | SPI1 RX | HAL DMA | 【已确认｜A级】 |
| 两个工程 | DMA | `DMA2_Stream2_IRQHandler` | `:441` | USART6 RX | HAL DMA | 【已确认｜A级】 |
| 两个工程 | DMA | `DMA2_Stream3_IRQHandler` | `:455` | SPI1 TX | HAL DMA | 【已确认｜A级】 |
| 两个工程 | DMA | `DMA2_Stream5_IRQHandler` | `:511` | USART1 RX | HAL DMA | 【已确认｜A级】 |
| 两个工程 | DMA | `DMA2_Stream6_IRQHandler` | `:525` | USART6 TX | HAL DMA | 【已确认｜A级】 |
| 两个工程 | DMA | `DMA2_Stream7_IRQHandler` | `:539` | USART1 TX | HAL DMA | 【已确认｜A级】 |
| 两个工程 | CAN RX | `CAN1_RX0_IRQHandler` | `:257` | CAN1 FIFO0 | `HAL_CAN_IRQHandler(&hcan1)` → CAN FIFO 回调 | 【已确认｜A级】 |
| 两个工程 | CAN RX | `CAN1_RX1_IRQHandler` | `:271` | CAN1 FIFO1 | 同上 | 【已确认｜A级】 |
| 两个工程 | CAN RX | `CAN2_RX0_IRQHandler` | `:469` | CAN2 FIFO0 | `HAL_CAN_IRQHandler(&hcan2)` | 【已确认｜A级】 |
| 两个工程 | CAN RX | `CAN2_RX1_IRQHandler` | `:483` | CAN2 FIFO1 | 同上 | 【已确认｜A级】 |
| 两个工程 | I²C | `I2C2_EV_IRQHandler` / `I2C2_ER_IRQHandler` | `:299/:313` | I2C2 | HAL event/error handler | 【已确认｜A级】 |
| 两个工程 | I²C | `I2C3_EV_IRQHandler` / `I2C3_ER_IRQHandler` | `:567/:581` | I2C3 | HAL event/error handler | 【已确认｜A级】 |
| 两个工程 | SPI | `SPI1_IRQHandler` / `SPI2_IRQHandler` | `:327/:341` | SPI1/SPI2 | `HAL_SPI_IRQHandler()` | 【已确认｜A级】 |
| 两个工程 | UART | `USART1_IRQHandler` | `:355` | USART1 | `HAL_UART_IRQHandler(&huart1)` | 【已确认｜A级】 |
| 两个工程 | UART | `USART3_IRQHandler` | `:369` | USART3 | `HAL_UART_IRQHandler(&huart3)` | 【已确认｜A级】 |
| 两个工程 | UART | `USART6_IRQHandler` | `:553` | USART6 | `HAL_UART_IRQHandler(&huart6)` | 【已确认｜A级】 |
| 两个工程 | Timer | `TIM8_TRG_COM_TIM14_IRQHandler` | `:383` | TIM8/TIM14 | 调两个 HAL TIM handler；TIM14 为 HAL 时基 | 【已确认｜A级】 |
| 两个工程 | Timer/DAC | `TIM6_DAC_IRQHandler` | `:412` | TIM6、DAC | HAL DAC + TIM handler | 【已确认｜A级】 |
| 两个工程 | USB | `OTG_FS_IRQHandler` | `:497` | USB OTG FS | `HAL_PCD_IRQHandler()` | 【已确认｜A级】 |

FreeRTOS 的 SVC/PendSV/SysTick 由端口层和配置宏接管，不在上述项目 `stm32f4xx_it.c` 显式定义数中。此处不把第三方端口实现重复计入“项目中断数量”。

### 7.2 HAL/BSP 回调

| 工程 | 类型 | 函数 | 文件 | 初步处理内容 | 后续调用 | 可信度 |
| --- | --- | --- | --- | --- | --- | --- |
| 两者 | CAN RX FIFO0/1 | `HAL_CAN_RxFifo0MsgPendingCallback`、`HAL_CAN_RxFifo1MsgPendingCallback` | `bsp/can/bsp_can.c:218-233` | 遍历 FIFO 报文、按注册 ID 分发 | 各 `CANInstance.can_module_callback` | 【已确认｜A级】 |
| 两者 | UART 空闲/DMA | `HAL_UARTEx_RxEventCallback` | `bsp/usart/bsp_usart.c:108-121` | 按 UART 实例调用模块回调并重启 ReceiveToIdle DMA | 遥控器、图传、裁判等模块回调 | 【已确认｜A级】 |
| 两者 | UART 错误 | `HAL_UART_ErrorCallback` | `bsp/usart/bsp_usart.c:133-141` | 重启 DMA 接收 | 同一 USART 实例 | 【已确认｜A级】 |
| 两者 | GPIO EXTI | `HAL_GPIO_EXTI_Callback` | `bsp/gpio/bsp_gpio.c:15` | 按 GPIO 实例分发回调 | 注册模块 | 【已确认｜A级】 |
| 两者 | SPI RX/TXRX | `HAL_SPI_RxCpltCallback`、`HAL_SPI_TxRxCpltCallback` | `bsp/spi/bsp_spi.c:156-184` | 调注册回调；TXRX 复用 RX 回调 | SPI 设备模块 | 【已确认｜A级】 |
| 两者 | I²C RX | `HAL_I2C_MasterRxCpltCallback`、`HAL_I2C_MemRxCpltCallback` | `bsp/iic/bsp_iic.c:129-150` | 按实例分发；Mem 回调复用 Master 回调 | I²C 设备模块 | 【已确认｜A级】 |
| 两者 | PWM | `HAL_TIM_PWM_PulseFinishedCallback` | `bsp/pwm/bsp_pwm.c:15` | PWM DMA/完成回调分发 | PWM 实例 | 【已确认｜A级】 |
| 两者 | Timer | `HAL_TIM_PeriodElapsedCallback` | `bsp/tim/bsp_tim.c:31` | 定时器实例回调并处理 TIM14 tick | 注册模块/HAL tick | 【已确认｜A级】 |
| 两者 | USB PCD | 11 个 `HAL_PCD_*Callback` | `Src/usbd_conf.c:135-312` | USB Device Core 状态和端点事件 | ST USB Device Library | 【已确认｜A级】 |
| 两者 | USB CDC RX | `CDC_Receive_FS` | `Src/usbd_cdc_if.c:262-269` | 调 `rx_cbk(*Len)` | USB BSP/上位机解码 | 【已确认｜A级】 |

每工程可确认 22 个具名 HAL 回调（11 个 BSP + 11 个 USB PCD）；`CDC_Receive_FS` 是 USB 类接口回调，单列但不计入 `HAL_*Callback` 数。

### 7.3 看门狗

`【已确认｜A级】` 没有发现硬件 IWDG/WWDG 初始化、IRQ 或喂狗调用；相应 HAL 模块宏被注释。项目中的 `Daemon` 是软件在线监测，不应称为硬件看门狗。

## 8. 主要设备与驱动索引

| 设备或模块 | 驱动文件 | 通信接口 | 主要结构体 | 初始化函数 | 数据更新入口 | 使用工程/活动性 | 可信度 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| RS 电机 | `modules/motor/RSmotor/rsmotor.c/.h` | CAN | `RSMotorInstance` | `RSMotorInit()` | CAN 注册回调；`RSMotorControl()` | 机械臂初始化 3 个（`gimbal.c:67,104,141`）；底盘初始化 3 个（`chassis.c:168,208,211`） | 【已确认｜A级】 |
| DM4310 类电机 | `modules/motor/DMmotor/dmmotor.c/.h` | CAN | `DMMotorInstance` | `DMMotorInit()` | CAN 回调 + 独立 `DMMotorTask` | 机械臂 2 个（`gimbal.c:180,217`）；底盘 2 个（`chassis.c:251,253`） | 【已确认｜A级】；具体商用品名仍依协议资料 |
| LK4005/LK 驱动 | 机械臂 `LKmotor/LKmotor.c`; 底盘库存为 `LK9025.c` | CAN | `LKMotorInstance` | `LKMotorInit()` | CAN 回调；`LKMotorControl()` | 机械臂初始化 2 个（`gimbal.c:253,287`）；底盘活动初始化未发现 | 【已确认｜A级】 |
| DJI 电机 | `modules/motor/DJImotor/dji_motor.c/.h` | CAN | `DJIMotorInstance` | `DJIMotorInit()` | CAN 回调；`DJIMotorControl()` | 底盘初始化 6 个（四轮 + 两履带，`chassis.c:95-127`）；机械臂活动初始化未发现 | 【已确认｜A级】 |
| PWM 舵机 | `modules/motor/servo_motor/servo_motor.c/.h` + `bsp/pwm` | TIM PWM | `ServoInstance` | `ServoInit()` | 任务中更新 PWM 比较值 | 机械臂图传 pitch/yaw 各 1 个（`gimbal.c:299,310`） | 【已确认｜A级】 |
| BMI088/INS | `modules/BMI088/`、`modules/imu/` | SPI1 + GPIO EXTI/PWM 加热线索 | `BMI088Instance`、`INS_t`、`attitude_t` | `BMI088Init()`/`INS_Init()` | `INS_Task()` → Quaternion EKF | 底盘活动（`chassis.c:58`; `ins_task.c:80-169`）；机械臂 INS 任务创建被注释 | 【已确认｜A级】 |
| 遥控器接收 | `modules/remote/remote_control.c/.h` | USART3，DMA+IDLE | `RC_ctrl_t` | `RemoteControlInit(&huart3)` | `RemoteControlRxCallback` → `wbus_to_rc/sbus_to_rc` | 两工程均初始化；机械臂活动消费，底盘本地消费分支被注释 | 【已确认｜A级】 |
| 图传键鼠/自定义数据 | `modules/referee/rm_referee.c/.h` | USART1，DMA+IDLE | `ImgTrans_info_t` | `ImgTransInit(&huart1)` | `ImgTransRxCallback` → 解包/`HandleKeyMouseData` | 机械臂活动；底盘初始化调用被注释 | 【已确认｜A级】 |
| 裁判系统/UI | `modules/referee/rm_referee.c`、`referee_task.c` | USART6 | `referee_info_t`、`Referee_Interactive_info_t` | `UITaskInit(&huart6, ...)` | UART 回调 + UI 任务 | 机械臂活动；底盘调用及 UI 任务被注释 | 【已确认｜A级】 |
| USB 上位机 | `modules/master_machine/host_communication.c` + `bsp/usb` | USB CDC | `Host_Recv_s`、`Host_Send_s` | `HostInit()` | `CDC_Receive_FS` → `DecodeHost` | 机械臂接收活动，但 `HostControlSet()` 调用被注释，当前不作为活动控制源 | 【已确认｜A级】 |
| 板间 CANComm | `modules/can_comm/can_comm.c/.h` | 机械臂 CAN1 ↔ 底盘 CAN2，标准 ID | `CANCommInstance`、`Master_Cmd_Data_s`、`Slave_Upload_Data_s` | `CANCommInit()` | CAN FIFO 回调 → `CANCommRxCallback` | 两工程活动；机械臂发 0x311/收 0x312，底盘发 0x312/收 0x311 | 【已确认｜A级】；第二阶段依据两端 `robot_cmd.c:121-132` 修正总线标注 |
| 软件在线监测 | `modules/daemon/daemon.c/.h` | 任务/内存 | `DaemonInstance` | `DaemonRegister()` | `DaemonReload()`、10 ms `DaemonTask()` | 两工程活动 | 【已确认｜A级】 |
| 气泵 | `modules/pump/pump.c/.h` | GPIO（从调用形态确认） | 状态字节 | `get_pump_state_ptr()`；`pump_run/stop` | `PumpControl/GrabberControlSet` | 驱动和命令辅助函数存在，但第二阶段未找到活动任务调用；相关键鼠泵调用已注释 | 【已确认｜A级】（当前非活动） |
| 独立编码器 | `modules/encoder/{AS5600,MT6701,MT6816}` | I²C/SPI（各驱动实现） | 各 `*Instance` | `AS5600Init/MT6701Init/MT6816Init` | 各驱动读函数/回调 | 驱动被 Makefile 编译，但活动业务初始化未找到 | 【已确认｜A级】（未找到活动调用） |
| HC05 蓝牙 | `modules/bluetooth/HC05.c/.h` | UART（驱动代码） | 模块静态数据 | 驱动接口 | UART | 被编译，活动初始化未找到 | 【已确认｜A级】（存在）；业务用途【代码中未确认｜D级】 |
| TFminiPlus、OLED、IST8310、超级电容 | 对应 `modules/` 子目录 | UART/I²C/CAN 等，按各驱动 | 各模块结构 | 各模块 Init | 各协议回调/轮询 | 代码存在；当前 `RobotInit` 活动链未完整确认 | 【推测｜C级】业务作用需第二阶段逐项验证 |

## 9. 控制算法索引

本节只建立“定义—调用”索引，不展开数学推导。

| 算法 | 定义文件 | 核心函数 | 调用位置 | 使用对象 | 当前能确认的作用 | 可信度 |
| --- | --- | --- | --- | --- | --- | --- |
| PID | `modules/algorithm/controller.c` | `PIDInit`、`PIDCalculate` | 各电机驱动；底盘 `chassis.c:265,412` | 电机角度/速度/电流环、底盘旋转 | 计算闭环输出 | 【已确认｜A级】 |
| 串级 PID | 各电机驱动 | 角度→速度→电流的条件链 | DJI `dji_motor.c:294-316`; RS `rsmotor.c:276-290`; DM `dmmotor.c:298-320` | 启用对应闭环的电机实例 | `pid_ref` 依次经过启用的控制环 | 【已确认｜A级】 |
| 通用 Kalman | `modules/algorithm/kalman_filter.c` | `Kalman_Filter_Init/Update` | `QuaternionEKF.c:68,189` | IMU EKF 的矩阵滤波核心 | 状态估计基础 | 【已确认｜A级】 |
| Quaternion EKF | `modules/algorithm/QuaternionEKF.c` | `IMU_QuaternionEKF_Init/Update` | 底盘 `modules/imu/ins_task.c:101,147` | BMI088 姿态 | 输出四元数、Yaw/Pitch/Roll | 【已确认｜A级】 |
| 加速度低通 | `QuaternionEKF.c:142-151` | EKF 更新内部一阶低通 | 同文件 | IMU 加速度 | 平滑量测 | 【已确认｜A级】 |
| 正/逆运动学 | `modules/algorithm/kinematics.c` | `Robotics_FK`、`ResolveInverseKinematics` | 两工程命令文件保留实现/调用痕迹，机械臂当前相关调用位于注释块；底盘本地命令分支整体不活动 | 机械臂关节/末端位姿 | 算法代码存在，但第二阶段没有找到进入当前活动机械臂或底盘电机链的调用 | 【已确认｜A级】（当前非活动） |
| 三次轨迹插值 | `modules/algorithm/trajectory_planning.c` | `CubicTrajectory_Init/Update` | `auto_action.c`; 两工程 `RobotCMDTask()` | 自动动作/自定义控制目标 | 随时间更新 `traj.pos` | 【已确认｜A级】 |
| 自动动作状态机 | `modules/fsm/auto_action.c` | `AutoActionInit` 及事件处理函数 | 两工程 `RobotCMDInit/RobotCMDTask` | 预设动作 | 管理动作事件并触发轨迹 | 【已确认｜A级】 |
| 麦克纳姆轮解算 | `Engineer_chassis/application/chassis/chassis.c` | `MecanumCalculate` | `ChassisTask()` 附近 `:454` | 四个底盘轮 | `vx/vy/wz` 到四轮速度 | 【已确认｜A级】；`chassis.c:288-294` |
| 功率限制 | `Engineer_chassis/modules/power_control/power_control.c` | `PowerControlInit`、`calfromPID`、`chassis_power_control` | `chassis.c:267-321` | 四个 DJI 底盘轮 | 预测功率并缩放速度设定 | 【已确认｜A级】 |
| 限幅/死区/角度归一化 | `modules/algorithm/user_lib.c` | `float_constrain`、`Math_Constrain`、`float_deadband`、`loop_float_constrain` | 命令与底盘/电机代码 | 目标值和角度 | 基础数值约束 | 【已确认｜A级】 |
| 简单低通/平均滤波 | `robot_cmd.c:419,454-455`; `user_lib.c:207` | 内联一阶滤波、`AverageFilter` | 自定义控制器路径/工具库 | 控制器输入 | 平滑输入；部分路径是否活动需看分支 | 【较可信｜B级】 |
| CRC8/CRC16 | `modules/algorithm/crc8.c`、`crc16.c`、`referee/crc_ref.c` | `crc_8` 等 | CANComm、上位机/裁判协议 | 通信帧 | 完整性校验 | 【已确认｜A级】 |
| 前馈、重力补偿、摩擦补偿 | 未找到可闭合的活动实现链 | — | — | — | 本阶段不能确认当前项目已采用 | 【代码中未确认｜D级】 |

## 10. 控制入口初步识别

只追踪到“首次写入控制目标或状态变量”。不在第一阶段继续追到电机输出。

| 输入来源 | 接收入口 | 解析函数 | 保存结构体 | 首次影响的控制变量 | 当前追踪终点 | 可信度 |
| --- | --- | --- | --- | --- | --- | --- |
| 机械臂遥控器 SBUS/WBUS | USART3 IRQ/DMA → `HAL_UARTEx_RxEventCallback` | `RemoteControlRxCallback` → `wbus_to_rc/sbus_to_rc` | `RC_ctrl_t rc_ctrl[2]` | `RemoteControlSet()` 写 `gimbal_cmd_send.target_arm_angle[]`、`chassis_cmd_send`、事件位 | `Engineer_arm/application/cmd/robot_cmd.c:271-401` | 【已确认｜A级】 |
| 机械臂图传键鼠 | USART1 IRQ/DMA → UART BSP 回调 | `ImgTransRxCallback`、裁判协议解包、`HandleKeyMouseData` | `ImgTrans_info_t` | `MouseKeySet()` 写机械臂/底盘目标、模式和事件 | `robot_cmd.c:956` 起 | 【已确认｜A级】 |
| 图传自定义控制器数据 | 同上，命令 `0x0302` 数据段 | `ImgTransRxCallback` | `ImgTrans_info.ControllerToRobot` | `CustomControllerSet()` 写六自由度轨迹与机械臂目标；顶层直接调用被注释，但键鼠 F 奇偶切换分支会调用 | 定义 `robot_cmd.c:413-499`；活动条件调用 `:1151-1170` | 【已确认｜A级】（有条件活动；第二阶段修正） |
| USB 上位机 | USB OTG FS/CDC → `CDC_Receive_FS` | `DecodeHost` | `Host_Recv_s recv_data` | `HostControlSet()` 设计为处理上位机命令，但活动调用被注释 | `host_communication.c:41-65`; `robot_cmd.c:506-528,921` | 【已确认｜A级】（接收活动、控制非活动） |
| 自动动作 | `RobotCMDTask()` 周期调用 | `AutoActionHandler` / FSM | `AutoActionInstance`、`CubicTrajectory traj` | 写 `gimbal_cmd_send.target_arm_angle[]` 或底盘目标 | 机械臂 `robot_cmd.c:898,905`; 底盘 `:602,604` | 【已确认｜A级】 |
| 机械臂板→底盘板命令 | 机械臂 CAN1 TX → 底盘 CAN2 RX → CAN BSP → `CANCommRxCallback` | CANComm 帧头/长度/CRC8 解包 | `Master_Cmd_Data_s master_cmd_recv` | 底盘 `RobotCMDTask()` 复制到 `chassis_cmd_send` | `Engineer_chassis/robot_cmd.c:570-593` | 【已确认｜A级】 |
| 底盘板→机械臂板反馈 | 反向 CANComm | 同上 | `Slave_Upload_Data_s slave_fetch_data` | 机械臂初始化/事件/急停逻辑读取底盘反馈 | `Engineer_arm/robot_cmd.c:891,912-915` | 【已确认｜A级】 |
| 底盘本地遥控器 | USART3 接收仍初始化 | 同遥控解析 | `RC_ctrl_t` | 本地 `RemoteControlSet()` 存在，但 `RobotCMDTask()` 调用分支被注释 | `Engineer_chassis/robot_cmd.c:103,204-391,606-616` | 【已确认｜A级】（当前非活动） |
| 底盘本地图传键鼠 | 初始化调用被注释 | 驱动存在 | `ImgTrans_info_t *img_data` | `MouseKeySet()` 存在但未由活动路径调用 | `Engineer_chassis/robot_cmd.c:107,644` 起 | 【已确认｜A级】（当前非活动） |
| 独立 HC05/标准命令/unicomm | 驱动/协议文件存在 | 各模块函数 | 各模块结构 | 未找到从 `RobotInit()` 到控制变量的闭合活动链 | 证据链中断 | 【推测｜C级】 |

机械臂活动输入的阶段性链路：

```text
USART3 遥控器 / USART1 图传键鼠 / 自动动作
→ 解析结构体
→ RobotCMDTask（5 ms 任务）
→ gimbal_cmd_send / master_cmd_data
→ PubPushMessage 或 CANCommSend
→ 第一阶段证据链止于此；第二阶段已在 `05-arm-control.md` 追踪到 7 路电机 CAN 命令与反馈闭环。
```

底盘活动输入的阶段性链路：

```text
机械臂 CAN1 → 底盘 CAN2 板间报文 0x311
→ CAN BSP 注册回调
→ CANComm CRC8 解包
→ Master_Cmd_Data_s
→ chassis_cmd_send
→ PubPushMessage("chassis_cmd")
→ 第一阶段证据链止于此；第二阶段已在 `06-chassis-control.md` 追踪到四轮、履带、姿态电机命令与反馈闭环。
```

## 11. 自研、生成与第三方代码分类

| 类别 | 典型路径 | 判定依据与耦合说明 | 可信度 |
| --- | --- | --- | --- |
| 启动和芯片初始化 | `startup_stm32f407xx.s`、`Src/system_stm32f4xx.c` | ST/CubeMX 启动框架 | 【已确认｜A级】 |
| CubeMX 自动生成 | `Engineer.ioc`、`Src/main.c`、`Src/*外设.c`、`Src/stm32f4xx_it.c`、`Src/freertos.c`、`Src/usbd_*`、`Inc/` | 文件头、`USER CODE` 区和 `.ioc` 函数清单共同支持 | 【已确认｜A级】 |
| CubeMX 与项目代码混合 | `Src/main.c`、`Src/usbd_cdc_if.c`、`Src/freertos.c` | 生成框架内有 `RobotInit()`、USB 自定义回调等项目修改；不能整文件视为纯生成代码 | 【已确认｜A级】 |
| HAL/CMSIS | `Drivers/STM32F4xx_HAL_Driver/`、`Drivers/CMSIS/` | 厂商路径、版权头和许可证 | 【已确认｜A级】 |
| FreeRTOS/CMSIS-RTOS | `Middlewares/Third_Party/FreeRTOS/` | bundled LICENSE、Amazon/Arm/ST 版权头 | 【已确认｜A级】 |
| 板级外设驱动 | `bsp/` | 项目封装；部分文件有独立版权头，来源并非全部可归为本项目原创 | 【较可信｜B级】 |
| 设备驱动 | `modules/motor/`、`encoder/`、`BMI088/`、`imu/`、`oled/` 等 | 项目/第三方混合；需逐文件看版权头 | 【已确认｜A级】（混合来源） |
| 通信协议 | `modules/remote/`、`referee/`、`can_comm/`、`master_machine/` | 有 DJI、HNU YueLu EC、项目修改及厂商协议混合 | 【已确认｜A级】 |
| 控制算法 | `modules/algorithm/`、`power_control/` | PID、Kalman/EKF、运动学、轨迹等；`controller.c` 有 HNU YueLu EC 声明 | 【已确认｜A级】 |
| 机械臂业务 | `Engineer_arm/application/gimbal/`、`application/cmd/`、`application/robot.c` | 活动调用链直接确认 | 【已确认｜A级】 |
| 底盘业务 | `Engineer_chassis/application/chassis/`、`application/cmd/`、`application/robot.c` | 活动调用链直接确认 | 【已确认｜A级】 |
| 调试和日志 | `bsp/log/`、SEGGER RTT、`.vscode/`、`*.jdebug*` | 日志 BSP、RTT 和调试配置 | 【已确认｜A级】 |
| 安全和故障处理 | `modules/daemon/`、各模块 Lost/Error callback、Fault handlers | 软件超时与错误入口；硬件看门狗未启用 | 【已确认｜A级】 |
| 第三方库 | ST USB、CMSIS-DSP、SEGGER RTT、FreeRTOS 等 | 目录与许可证直接确认 | 【已确认｜A级】 |
| 编译产物 | `build/` | Makefile 目标及扩展名 | 【已确认｜A级】 |
| 尚未分类/待溯源 | 无明确文件头的部分 `application/bsp/modules` 文件、厂商 PDF/PNG | 仓库级 Apache-2.0 只覆盖有权授权的原创贡献，不能自动覆盖来源不明材料 | 【代码中未确认｜D级】 |

## 12. 编译产物说明

`【已确认｜A级】` 两工程现有 `build/` 均包含完整固件产物和中间文件：

| 工程 | `.o` | `.d` | `.lst` | `.elf/.hex/.bin/.map` |
| --- | ---: | ---: | ---: | ---: |
| 机械臂 | 133 | 133 | 131 | 各 1 |
| 底盘 | 134 | 134 | 132 | 各 1 |

这些文件可辅助确认历史构建范围，但不是核心源码分析对象，也不能证明当前源码可重现这些二进制。Makefile 的 `clean` 目标会递归删除 `build/`；本阶段没有执行 `clean`，也没有修改或删除任何现有构建产物。

## 13. 许可证和版权初步扫描

本节只做工程识别，不构成法律意见。

| 路径或组件 | 版权方 | 许可证或声明 | 是否允许确认 | 风险说明 |
| --- | --- | --- | --- | --- |
| 仓库根 `LICENSE` / `NOTICE` | 26-EngBot contributors + 多方 | 多许可证总说明 | 是 | 明确声明仓库不是不可分割的单一许可作品 |
| `Firmware/LICENSE` | 26-EngBot contributors | 原创固件贡献 Apache-2.0 | 是，但仅限有权授权的原创部分 | 明确排除 CubeMX/ST、CMSIS、FreeRTOS、SEGGER、独立声明文件和厂商资料 |
| `Drivers/CMSIS/` | Arm Limited / ST 等 | Apache-2.0（bundled `LICENSE.txt` 与 SPDX 头） | 是 | 需保留版权和许可证 |
| `Drivers/STM32F4xx_HAL_Driver/` | STMicroelectronics | bundled 文件说明：包许可；缺包许可时 BSD-3-Clause | 可确认文件声明 | 应保留 ST 文件头和许可证；不能由项目许可证覆盖 |
| `Middlewares/Third_Party/FreeRTOS/Source/` | Amazon.com, Inc. or affiliates | MIT | 是 | 需保留版权和许可文本 |
| `Middlewares/ST/STM32_USB_Device_Library/` | STMicroelectronics | SLA0044 | 是 | 使用/执行限制于 ST 制造或为 ST 制造的 MCU/MPU；不是普通开源许可 |
| `Middlewares/Third_Party/SEGGER/` | SEGGER Microcontroller GmbH | 各文件内再分发声明、`All rights reserved` | 只能确认声明存在 | 没有独立 LICENSE 文件；再分发必须逐文件核对条款 |
| `modules/algorithm/controller.c`、`motor_def.h`、多个电机/遥控文件 | HNU YueLu EC、DJI 等 | 多处 `all rights reserved` 或独立版权 | 是（声明存在） | `Firmware/LICENSE` 明确不重许可这些文件 |
| `modules/fsm/*` | SPR / 2026 SPR | 文件头版权 | 是（声明存在） | 许可文本未在文件头明确给出，需后续溯源 |
| 电机/传感器协议 PDF、PNG、datasheet | 各厂商/发布者 | 厂商资料 | 部分可识别 | 默认不随项目 Apache-2.0 重许可 |
| `Middlewares/ST/ARM/DSP/Lib/*.a` | Arm/ST | 预编译库 + CMSIS-DSP 相关声明 | 初步确认 | 二进制可再分发条件需结合随附许可证核对 |

初步风险：

1. `application/`、`bsp/`、`modules/` 不是“全部项目原创”的同质目录，必须按文件头和来源逐项判断。
2. `all rights reserved` 文件不能因位于 `Firmware/` 下就自动视为 Apache-2.0。
3. ST USB 的 SLA0044 有设备使用限制，发布固件或移植到非 ST 芯片前必须单独核对。
4. SEGGER RTT、厂商手册和预编译库需要保留原声明并核实分发条件。

## 14. 当前证据等级分布与不确定性

本阶段索引以 A 级为主：构建、入口、任务创建、IRQ、回调、外设、活动初始化和板间 CAN ID 都能由直接代码确认。B/C/D 主要集中在物理业务映射、未调用驱动、硬件型号和许可权属。

| 等级 | 主要覆盖内容 | 本阶段处理方式 |
| --- | --- | --- |
| A | 实际编译入口、任务创建、外设、IRQ/回调、活动初始化、CAN ID、构建目标 | 写成确定代码事实 |
| B | `gimbal` 与物理机械臂命名关系、气泵物理用途、两工程同源关系 | 写明推理与缺失材料 |
| C | 未进入活动链的 HC05、TFminiPlus、独立编码器等业务用途 | 只列为驱动库存，不写成当前功能 |
| D | PCB/接线、物理方向、正式操作方式、实机安全、来源不明文件许可权 | 进入统一待确认清单 |

最大不确定性包括：

- `Core/` 与顶层 `Src/Inc` 的生成来源、同步规则和后续应保留哪一套尚未由仓库文档说明。
- 两工程共享大量代码却各自编译全部业务文件；“被编译”与“被 `RobotInit()` 初始化”必须持续区分。
- 机械臂工程 USB 上位机接收已启用但控制调用被注释；底盘本地遥控器接收已启用但活动控制分支被注释。
- 实际遥控器型号、开关方向、关节编号、电机物理方向、限位和急停装置均缺硬件资料。
- 没有交叉编译器，未验证当前源码能否从零重建；现有 `build/` 不能替代可复现构建。

## 15. 后续重点阅读文件

按第二阶段优先级排序：

1. `Firmware/Engineer_arm/application/cmd/robot_cmd.c`：遥控器、图传键鼠、自动动作、急停、限幅与板间命令汇合点。
2. `Firmware/Engineer_arm/application/gimbal/gimbal.c`：机械臂命令到各电机参考值和反馈发布的业务层。
3. `Firmware/Engineer_arm/modules/remote/remote_control.c`：USART3 原始帧到 `RC_ctrl_t`。
4. `Firmware/Engineer_arm/modules/referee/rm_referee.c`：图传键鼠/自定义控制器数据入口。
5. 两工程 `modules/can_comm/can_comm.c` 与 `application/robot_def.h`：板间帧、数据结构、事件位与方向。
6. `Firmware/Engineer_chassis/application/cmd/robot_cmd.c`：板间命令到 `chassis_cmd_send`。
7. `Firmware/Engineer_chassis/application/chassis/chassis.c`：底盘命令、麦克纳姆解算、功率限制和各电机参考值。
8. 两工程 `modules/motor/motor_task.c` 与当前活动电机驱动：电机控制任务、PID 和 CAN 输出。
9. 两工程 `bsp/can/bsp_can.c`、`bsp/usart/bsp_usart.c`：中断/回调分发边界。
10. `Firmware/Engineer_chassis/modules/imu/ins_task.c` 与 `modules/algorithm/QuaternionEKF.c`：BMI088 到姿态反馈。

## 16. 第一阶段结论

`【已确认｜A级】` 仓库包含两个可独立构建的 STM32F407 固件工程，它们共用相似的 HAL/BSP/模块架构，但活动业务不同：机械臂工程启用命令与 `Gimbal` 业务，底盘工程启用命令与 `Chassis` 业务。两者通过机械臂 CAN1 与底盘 CAN2 的 `0x311/0x312` 互补 ID 交换主命令与底盘反馈。

第一阶段已建立入口、构建、任务、中断、回调、外设、设备、算法、输入、代码来源、编译产物和许可证索引。第二阶段已闭合“机械臂遥控/图传 → 机械臂电机”和“机械臂板 CAN → 底盘板 → 底盘电机”两条静态端到端链路，详见 `02-system-architecture.md`、`05-arm-control.md` 与 `06-chassis-control.md`。两个阶段均未修改控制代码、参数或 `.ioc`，未执行清理、下载或烧录。
