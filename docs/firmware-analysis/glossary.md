# 固件术语表

本术语表只收录当前仓库中可以定位到代码实体或通用技术定义的术语。业务含义无法由代码确认时，会明确保留原名或标注待确认。

| 术语 | 当前文档中的含义 | 代码依据 | 可信度 |
| --- | --- | --- | --- |
| Arm | 本仓库机械臂固件工程 `Firmware/Engineer_arm/`；业务代码仍大量沿用 `gimbal` 命名 | `Engineer_arm/application/robot.c:27-29` 启用 `GimbalInit()`、注释 `ChassisInit()` | 【已确认｜A级】 |
| Chassis | 底盘固件工程或底盘业务模块 | `Engineer_chassis/application/robot.c:27-29` 启用 `ChassisInit()` | 【已确认｜A级】 |
| Gimbal | 代码中的机械臂业务模块名称，不应直接翻译为本项目存在独立“云台”机构 | `Engineer_arm/application/gimbal/gimbal.c:27` 的 `GimbalInit()` 初始化多类机械臂电机；物理命名仍待装配资料确认 | 【较可信｜B级】 |
| Joint | 机械臂关节概念。代码用 `target_arm_angle[]`、`feedback_arm_angle[]` 表示多个关节/执行器通道，但物理关节编号映射尚未确认 | `Engineer_arm/application/cmd/robot_cmd.c:320-338,861-867` | 【较可信｜B级】 |
| Motor | 电机或电机驱动实例；仓库包含 DJI、DM、RS、LK、HT、YS、步进和舵机驱动 | `modules/motor/`；实际初始化见机械臂 `gimbal.c:67-310` 与底盘 `chassis.c:95-253` | 【已确认｜A级】 |
| Encoder | 编码器或电机反馈中的角度编码值。独立驱动包括 AS5600、MT6701、MT6816，但当前活动业务初始化中未找到这三者的调用 | `modules/encoder/`; `AS5600Init`、`MT6701Init`、`MT6816Init` | 【已确认｜A级】 |
| Target / Ref | 控制目标或参考值，常见字段为 `Ref`、`target_arm_angle`、`target_chassis_angle` | `modules/motor/motor_def.h`; 两工程 `application/cmd/robot_cmd.c` | 【已确认｜A级】 |
| Feedback / Measure | 反馈或测量值，常见字段为 `measure`、`feedback_arm_angle`、`feedback_chassis_angle` | `modules/motor/motor_def.h`; 两工程 `application/robot_def.h` | 【已确认｜A级】 |
| PID | 比例—积分—微分控制器；本仓库统一实现为 `PIDInstance`、`PIDInit()`、`PIDCalculate()` | 两工程 `modules/algorithm/controller.c:128-147` 附近；具体工程行号相差约 2 行 | 【已确认｜A级】 |
| Cascaded PID / 串级 PID | 多个 PID 环按配置顺序连接。各电机驱动可按角度、速度、电流环逐层计算；具体启用组合取决于实例配置 | `modules/motor/DJImotor/dji_motor.c:294-316`、`RSmotor/rsmotor.c:276-290` 等 | 【已确认｜A级】 |
| CAN | 控制器局域网。两工程配置 CAN1/CAN2 为 1 Mbit/s，并由 BSP 回调分发报文 | 两个 `Engineer.ioc:12-29`; `bsp/can/bsp_can.c:176-228` | 【已确认｜A级】 |
| CANComm | 项目封装的多帧 CAN 通信模块，带 `'s'` 帧头、长度、CRC8、`'e'` 帧尾和守护实例；板间实际连接为机械臂 CAN1 ↔ 底盘 CAN2 | `modules/can_comm/can_comm.h:20-22`; 两端 `application/cmd/robot_cmd.c:121-132` | 【已确认｜A级】 |
| Master / Slave | 板间通信数据方向名称：机械臂工程发送 `Master_Cmd_Data_s`，底盘工程返回 `Slave_Upload_Data_s`。它描述软件协议角色，不等同于 CAN 硬件主从关系 | 两工程 `application/cmd/robot_cmd.c:121-132` | 【已确认｜A级】 |
| UART / USART | 异步串行接口；本项目配置 USART1、USART3、USART6，并通过 DMA + 空闲事件接收 | `Engineer.ioc:50-58,586-597`; `bsp/usart/bsp_usart.c:31-35,108-140` | 【已确认｜A级】 |
| USB CDC / VCP | USB Device CDC 虚拟串口。默认任务调用 `MX_USB_DEVICE_Init()`，机械臂工程的 `HostInit()` 通过 USB BSP 注册接收回调 | `Src/freertos.c:122-130`; `modules/master_machine/host_communication.c:52-65` | 【已确认｜A级】 |
| SPI | 同步串行接口；两个 `.ioc` 均配置 SPI1、SPI2，BSP 支持阻塞、中断和 DMA 传输 | `Engineer.ioc:547-561`; `bsp/spi/bsp_spi.c` | 【已确认｜A级】 |
| I²C / IIC | 两线串行总线。工程同时使用 `I2C`（HAL 名称）和 `IIC`（BSP 目录/类型名称） | `Engineer.ioc:136-139`; `bsp/iic/bsp_iic.c` | 【已确认｜A级】 |
| PWM | 脉宽调制输出；BSP 封装定时器 PWM，机械臂工程用 `ServoInit()` 初始化两个舵机实例 | `bsp/pwm/bsp_pwm.c`; `Engineer_arm/application/gimbal/gimbal.c:299-310` | 【已确认｜A级】 |
| IMU | 惯性测量单元。底盘活动初始化路径使用 BMI088 数据与姿态解算；物理安装方向仍待确认 | `Engineer_chassis/application/chassis/chassis.c:58`; `modules/imu/ins_task.c:80-169` | 【已确认｜A级】 |
| INS | 本项目的惯性姿态解算模块，入口为 `INS_Init()` / `INS_Task()`，输出 `attitude_t` | `modules/imu/ins_task.h:70-92`; `ins_task.c:80,119` | 【已确认｜A级】 |
| Task | FreeRTOS/CMSIS-RTOS 线程。工程使用 `osThreadDef` 和 `osThreadCreate`，不能把所有名为 `*Task()` 的普通函数都自动视为独立线程 | `application/robot_task.h:37-57`; `Src/freertos.c:83-107` | 【已确认｜A级】 |
| Callback | 中断或驱动完成后由 HAL/BSP 调用的回调，例如 CAN FIFO、UART 空闲接收、GPIO EXTI | `bsp/can/bsp_can.c:218-228`; `bsp/usart/bsp_usart.c:108-140`; `bsp/gpio/bsp_gpio.c:15` | 【已确认｜A级】 |
| Daemon | 项目自研的软件在线监测模块，以周期计数和回调处理遥控器、CANComm 等模块离线；不是硬件独立看门狗 | `modules/daemon/daemon.c`; `remote_control.c:148-162`; `can_comm.c:86-113` | 【已确认｜A级】 |
| Watchdog | STM32 IWDG/WWDG 硬件看门狗。本仓库当前 HAL 配置中两者均未启用 | 两工程 `Inc/stm32f4xx_hal_conf.h:58,72` | 【已确认｜A级】 |
| Message Center | 项目自研的内存发布/订阅机制，使用 `PubRegister`、`SubRegister`、`PubPushMessage`、`SubGetMessage`；不是 FreeRTOS Queue | `modules/message_center/message_center.c`; `application/cmd/robot_cmd.c:112-116,888,926` | 【已确认｜A级】 |
| Control mode | 控制模式枚举与状态字段。机械臂活动模式为 STOP/ENABLE；底盘为 STOP/ENABLE/FOLLOW/ROTATE | 两工程 `application/robot_def.h:96-109`; `application/cmd/robot_cmd.c`; 底盘 `chassis.c:392-420` | 【已确认｜A级】 |
| AutoAction | 自动动作状态机实例，配合三次轨迹插值更新目标 | `modules/fsm/auto_action.c`; `modules/algorithm/trajectory_planning.c`; `RobotCMDInit()` | 【已确认｜A级】 |
| WBUS / SBUS | 遥控器串行帧的两种解析路径；当前回调根据首字节和第 24 字节选择，WBUS 解析 J1—J4 与 SA—SD | `modules/remote/remote_control.c:35-135` | 【已确认｜A级】 |
| J1—J4 | WBUS 代码中的四个 11-bit 操作通道；依次映射到 `rocker_r_`、`rocker_r1`、`rocker_l1`、`rocker_l_`。不在缺少遥控器资料时擅自替换成物理摇杆名称 | `remote_control.c:97-116` | 【已确认｜A级】 |
| SA—SD | WBUS 代码中的四个开关/模拟通道。解析后 down `>10`、mid `[-10,10]`、up `<-10`；物理拨杆方向待确认 | `remote_control.h:45-47,106-109` | 【已确认｜A级】 |
| FOLLOW | 底盘跟随模式：用 `arm_04_0` 反馈角做平移坐标旋转，并用 PID 令该相对角趋近 0 | `Engineer_chassis/application/chassis/chassis.c:336-349,410-414` | 【已确认｜A级】 |
| ROTATE | 底盘旋转模式：平移量仍经坐标旋转，旋转控制量固定写为 4000；该数值未标定为物理角速度 | `Engineer_chassis/application/chassis/chassis.c:415-418` | 【已确认｜A级】 |
| MecanumCalculate | 将 `vx/vy/wz` 组合为左前、右前、左后、右后四个速度目标的函数。函数名支持麦克纳姆意图，但真实轮型仍需硬件资料确认 | `Engineer_chassis/application/chassis/chassis.c:288-294` | 【较可信｜B级】 |
| Trajectory / Cubic trajectory | 六自由度三次多项式轨迹；输出 `traj.pos[6]` 和 `traj.vel[6]`，当前只把位置送入机械臂目标 | `modules/algorithm/trajectory_planning.h:13-32`; `.c:36-91` | 【已确认｜A级】 |
| Software STOP | 由遥控器在线状态/SA 和模块状态设置的电机 Stop 模式；属于软件命令，不等价于电源级急停或安全继电器 | 机械臂 `application/cmd/robot_cmd.c:543-612`; `gimbal.c:352-413` | 【已确认｜A级】 |
| Hardware emergency stop | 独立于 MCU、RTOS 和 CAN 的动力切断/安全回路；本仓库未提供其电气证据，不能与 Software STOP 混同 | `11-safety-analysis.md` 待确认 U-03 | 【代码中未确认｜D级】 |
| Stale command | 通信已超时但接收端仍沿用最后一次有效命令；当前 CANComm 离线回调只复位分包状态 | 两工程 `modules/can_comm/can_comm.c:73-78,126-135` | 【已确认｜A级】 |
| Deadline miss | 周期任务没有在要求时间内完成；当前项目未记录该指标，且相对 `osDelay()` 会形成漂移 | 两工程 `application/robot_task.h`、`11-safety-analysis.md` | 【较可信｜B级】 |
| SBOM | Software Bill of Materials，记录每个组件的版本、来源、许可证和 hash；仓库当前没有完整逐文件 SBOM | `15-license-audit.md` | 【代码中未确认｜D级】 |
| P0/P1/P2/P3 | 本文档的技术问题等级：P0 人身/碰撞/严重损坏，P1 失控/错误输出/崩溃，P2 稳定性/实时性/维护性，P3 一般改进 | `14-code-quality-review.md` | 【已确认｜A级】 |
| Event bitmask | `Master_Cmd_Data_s.event` / `Slave_Upload_Data_s.event` 的 1 B 位图，用于履带、底盘复位/爬台、初始化完成和 FSM busy 握手 | 两工程 `application/robot_def.h:237-245` 附近 | 【已确认｜A级】 |
| `motor_init_finish` | 应用命令中的初始化完成标志，用于决定是否允许正常模式；当前两板初始化函数使用未初始化的局部计数变量，不能把它视为可靠就绪证明 | 两工程 `application/cmd/robot_cmd.c:815-856` / `516-543` | 【已确认｜A级】 |
| CubeMX-generated | STM32CubeMX 生成或维护的启动、外设初始化、IRQ 框架及 USB 文件；`USER CODE` 区域可能混有项目修改 | `Engineer.ioc`; `Src/main.c`; `Src/stm32f4xx_it.c`; `Src/freertos.c` | 【已确认｜A级】 |
| BSP | Board Support Package，本仓库对 CAN/UART/SPI/I²C/GPIO/PWM/USB/DWT/日志等 HAL 能力的项目封装 | `bsp/` 与 `bsp/bsp_init.h:16-20` | 【已确认｜A级】 |
| HAL | STMicroelectronics STM32 Hardware Abstraction Layer | `Drivers/STM32F4xx_HAL_Driver/` 与其 `LICENSE.txt` | 【已确认｜A级】 |
| CMSIS | Arm Cortex 微控制器软件接口标准与设备头文件 | `Drivers/CMSIS/` 与其 `LICENSE.txt` | 【已确认｜A级】 |
| Build artifact | 编译生成物，如 `.o`、`.d`、`.lst`、`.elf`、`.hex`、`.bin`、`.map`；不作为核心源码逐文件分析 | 两工程 `Makefile:33,318-361` 附近及现有 `build/` | 【已确认｜A级】 |
| ISR / IRQ Handler | 中断服务入口；本项目的 CAN/UART/DMA/SPI/EXTI/USB IRQ 通常先调用 HAL，再到 BSP/设备回调 | 两工程 `Src/stm32f4xx_it.c`、`bsp/can/bsp_can.c` | 【已确认｜A级】 |
| DMA | 外设和内存之间的数据搬运机制；本项目 UART 使用 Receive-to-Idle DMA，DMA 本身不保证帧完整或共享结构一致 | `bsp/usart/bsp_usart.c:31-35,108-140` | 【已确认｜A级】 |
| DWT | Cortex-M Data Watchpoint and Trace 单元；本项目用其周期计数测 PID、轨迹和 INS 的实际 `dt` | `bsp/dwt/bsp_dwt.c`；`PIDCalculate()` | 【已确认｜A级】 |
| Relative delay | 每轮执行完再 `osDelay(N)`；实际周期包含执行时间和调度延迟，可能漂移 | 两工程 `application/robot_task.h` | 【已确认｜A级】 |
| Anti-windup / 抗积分饱和 | 执行器饱和且误差继续推动同方向饱和时抑制积分，并限制 `Iout` | `modules/algorithm/controller.c:38-62` | 【已确认｜A级】 |
| Derivative on measurement | PID 微分项使用测量变化而非误差变化，以减轻目标阶跃的微分冲击 | `controller.c:64-73,176-184` | 【已确认｜A级】 |
| EKF | 扩展卡尔曼滤波；活动底盘 INS 用陀螺/加速度更新四元数、零偏和姿态 | `modules/algorithm/QuaternionEKF.c:50-197`；`modules/imu/ins_task.c:147` | 【已确认｜A级】 |
| Kinematics | 坐标/关节之间的几何映射。底盘四轮解算活动；机械臂 FK/IK 实现存在但当前调用被注释 | chassis `chassis.c:288-294`；arm `modules/algorithm/kinematics.c` | 【已确认｜A级】 |
| HardFault | Cortex-M 严重故障异常；当前 Handler 只无限循环，没有主动保存 fault 寄存器 | 两工程 `Src/stm32f4xx_it.c` | 【已确认｜A级】 |
| ELF | 带分段、符号和调试信息的链接输出，用于 GDB/Cortex-Debug/Ozone | 两工程 `Makefile`、`.vscode/launch.json` | 【已确认｜A级】 |
| Intel HEX | 带目标地址记录的烧录文本格式 | 两工程 `Makefile` 的 `objcopy -O ihex` | 【已确认｜A级】 |
| BIN | 不带地址的原始固件字节流；仓库 OpenOCD 下载目标另指定 `0x08000000` | 两工程 `Makefile` | 【已确认｜A级】 |

## 暂不扩写的缩写

下列名称在代码中存在，但当前分析仍不根据名称自行补全业务含义：`RS`、`DM`、`LK`、`HT`、`YS`、`DJI` 各电机型号前缀，`SB/SC/SD` 开关标签，以及 `J1`—`J4` 通道。后续需要电机说明书、遥控器型号或装配资料交叉验证。
