# 初学者学习路线

> 目标不是“尽快让机器人动”，而是能用代码证据解释一条完整控制链，并在断能条件下判断数据是否正确。所有推荐实验默认离线、仿真、只读调试或断开动力；实机使能只允许有经验人员执行。

## 1. 十六阶段路线

| 阶段 | 阅读文件 | 核心函数 | 应掌握知识 | 推荐实验 | 完成标准 |
| ---: | --- | --- | --- | --- | --- |
| 1 工程目录与入口 | `README.md`、`01-repository-map.md`、两工程 `Src/main.c` | `main()` | 两个固件工程、自研/生成/第三方边界 | 画出三层目录树；不编译源码 | 能指出两板入口和业务目录 |
| 2 STM32 启动 | `startup_stm32f407xx.s`、`system_stm32f4xx.c` | `Reset_Handler()`、`SystemInit()` | 向量表、栈、`.data/.bss`、C 运行时 | 用 ELF/map 查 `Reset_Handler` 到 `main` | 能解释复位为何最终进入 main |
| 3 GPIO 与时钟 | `Src/main.c`、`Src/gpio.c`、`.ioc` | `HAL_Init()`、`SystemClock_Config()`、`MX_GPIO_Init()` | HSE/PLL、AHB/APB、GPIO 模式 | 用 CubeMX 只读打开 `.ioc` 并核对 168 MHz | 能算出 SYSCLK/APB1/APB2 |
| 4 UART 与 DMA | `bsp/usart/bsp_usart.c`、`remote_control.c` | `USARTRegister()`、`HAL_UARTEx_RxEventCallback()` | DMA、空闲事件、帧长、缓冲 | 用离线 25 B 帧调用/手算解析 | 能说明 DMA 与解析的边界 |
| 5 CAN | `bsp/can/bsp_can.c`、`07-communication-protocols.md` | `CANRegister()`、HAL FIFO callbacks | 标准/扩展 ID、DLC、过滤、1 Mbit/s | 离线解一帧反馈；不连接电机动力 | 能从 ID 找到对应设备实例 |
| 6 定时器/PWM | `bsp/pwm/bsp_pwm.c`、arm `gimbal.c` | `PWMRegister()`、`PWMSetDutyRatio()` | 定时器频率、compare、duty | 仅计算 0.02 s 舵机周期对应关系 | 能区分 PWM duty 与电机 CAN 命令 |
| 7 中断与回调 | `Src/stm32f4xx_it.c`、`03-startup-and-rtos.md` | IRQHandler → HAL → BSP callback | 中断上下文、执行时长、共享数据 | 画 CAN/UART 两条调用栈 | 能说出哪些解析发生在 ISR |
| 8 FreeRTOS | `Src/freertos.c`、`application/robot_task.h` | `OSTaskInit()`、各 Task loop | 优先级、栈、tick、相对延时 | 根据 `osDelay` 推导名义周期并说明误差 | 能列出每板六个活动任务 |
| 9 遥控器解析 | `remote_control.h/.c`、`04-operator-guide.md` | `wbus_to_rc()`、`RemoteControlRxCallback()` | 位域、偏置、值域、daemon | 断能读取 J1..J4/SA..SD 原值 | 能解释为何不擅自命名物理摇杆 |
| 10 电机反馈 | 四类电机驱动、`09-hardware-drivers.md` | 各 `Decode*` callback | 端序、范围压缩、回绕、单位 | 离线验证 DJI 8192 与 LK 65536 回绕 | 能从 CAN 字节得到 measure |
| 11 PID | `controller.h/.c`、`08-control-algorithms.md` | `PIDInit()`、`PIDCalculate()` | P/I/D、dt、饱和、对测量微分 | 用表格手算 3 个采样；不连电机 | 能解释串级和 MaxOut 单位 |
| 12 底盘控制 | chassis `robot_cmd.c`、`chassis.c` | `ChassisTask()`、`MecanumCalculate()` | 模式、坐标旋转、轮速映射 | 用四组 `(vx,vy,wz)` 离线算轮速符号 | 能解释 FOLLOW/ROTATE 的差别 |
| 13 机械臂控制 | arm `robot_cmd.c`、`gimbal.c` | `RobotCMDTask()`、`GimbalTask()` | 七通道、限位、串级、STOP | 在纸面追踪一个 joint 的目标到 CAN | 能给出 joint_0..6 控制类型 |
| 14 运动学 | `kinematics.c/.h`、`08-control-algorithms.md` | `ForwardKinematics()`、`InverseKinematics()` | 坐标系、几何解、奇异/不可达 | 离线输入安全数值并检查 FK/IK；不接入控制 | 能明确“代码存在但未活动” |
| 15 轨迹规划 | `trajectory_planning.c`、`auto_action.c` | `compute_segment_time()`、`CubicTrajectory_Update()` | 边界条件、速度/加速度、状态机 | 离线画单段 q/v/a；禁止实机自动动作 | 能发现 `fsm.size` 越界风险 |
| 16 状态机与安全 | 两板 `EmergencyHandler()`、daemon、`11-safety-analysis.md`、`14-code-quality-review.md` | mode/event/lost callbacks | 失联、优先级、故障覆盖、证据链 | 为遥控器/CAN/单电机失联各画时间线 | 能证明 STOP 是否覆盖每条输出路径，并指出 5 个 P0 |

## 2. 八条最有教学价值的实际代码路径

### 路径 A：复位到六个任务

```text
Reset_Handler
→ SystemInit / C 运行时
→ main
→ HAL_Init / SystemClock_Config / MX_*
→ RobotInit
→ OSTaskInit + MX_FREERTOS_Init
→ osKernelStart
→ 各任务循环
```

**代码事实：** 两工程启动汇编、`Src/main.c:80-142`、`application/robot.c:18-32`、`application/robot_task.h` 和 `Src/freertos.c:83-130` 构成这条链。【已确认｜A级】

**原理解释：** 外设在调度器前初始化，任务创建分散在 `RobotInit()` 和 CubeMX 的 FreeRTOS 初始化中；函数名带 `Task` 不必然是独立线程。

**改进建议：** 初学者先给每一步标“中断关闭/开启、调度器未启动/已启动”，能避免在 Init 中错误使用需要调度器的等待机制。

### 路径 B：UART DMA → 遥控器 → STOP/目标

```text
USART3 DMA/Idle
→ HAL_UARTEx_RxEventCallback
→ RemoteControlRxCallback
→ wbus_to_rc / sbus_to_rc
→ RC_ctrl_t
→ RobotCMDTask
→ EmergencyHandler / 目标生成
```

**代码事实：** 入口见 `bsp/usart/bsp_usart.c:108-140` 和 `modules/remote/remote_control.c:35-162`；命令消费见两板 `application/cmd/robot_cmd.c`。【已确认｜A级】

**原理解释：** ISR 是生产者，5 ms 命令任务是消费者，daemon 是新鲜度监督者；三者没有 FreeRTOS queue 隔离。

**改进建议：** 用录制帧验证位运算和值域，避免第一次学习就接动力。

### 路径 C：CAN 接收 → 电机反馈 → PID → CAN 发送

```text
CANx_RX_IRQHandler
→ HAL_CAN_IRQHandler
→ HAL_CAN_RxFifo*MsgPendingCallback
→ 设备 Decode 回调
→ motor.measure
→ MotorTask / DMTask
→ PIDCalculate
→ 协议打包
→ HAL_CAN_AddTxMessage
```

**代码事实：** IRQ 位于 `Src/stm32f4xx_it.c`，分发位于 `bsp/can/bsp_can.c:176-228`，四类电机控制位于各 `modules/motor/*`。【已确认｜A级】

**原理解释：** 反馈更新与控制计算运行在不同上下文；PID 的输入单位取决于设备解析，输出单位取决于协议打包。

**改进建议：** 先选一台 DJI 或 LK 做离线帧练习，再读更复杂的 RS/DM 范围压缩。

### 路径 D：机械臂输入 → 七通道目标 → 电机命令

```text
RC / 键鼠 / 自定义控制器 / 自动事件
→ RobotCMDTask
→ AutoActionHandler（可选）
→ EmergencyHandler
→ MotorAngleLimit
→ PubPushMessage(gimbal_cmd)
→ GimbalTask
→ MotorSetRef × 7
```

**代码事实：** 顺序见 `Engineer_arm/application/cmd/robot_cmd.c:884-926`，分发见 `application/gimbal/gimbal.c:352-439`。【已确认｜A级】

**原理解释：** 安全模式在目标生成之后覆盖模式，但 `GrabberAction()` 又能改变 joint_6 目标；最终停止依赖 GimbalTask 的 STOP 路径，不只看目标值。

**改进建议：** 练习时给每个写目标的位置编号，按实际执行顺序判断“最后写入者”。

### 路径 E：机械臂板命令 → CANComm → 底盘动作

```text
Master_Cmd_Data_s
→ CANCommSend
→ 分片 + CRC8
→ 底盘 CAN 回调/重组
→ Slave_cmd_recv
→ 底盘 RobotCMDTask
→ chassis_cmd_pub / 关节目标
```

**代码事实：** 两端初始化见 `application/cmd/robot_cmd.c:121-132`，封装见 `modules/can_comm/can_comm.c`，结构定义见 `application/robot_def.h`。【已确认｜A级】

**原理解释：** CAN 硬件没有“主从”；Master/Slave 是应用数据方向名。多帧协议要同时满足顺序、长度、CRC 和超时。

**改进建议：** 用抓包离线重组 21 B 与 41 B 载荷，禁止向真机重放命令。

### 路径 F：BMI088 → EKF → 姿态发布

```text
INSTask
→ BMI088_Read
→ 安装旋转
→ IMU_QuaternionEKF_Update
→ Euler / YawTotalAngle
→ degree-to-radian
→ VisionSend / attitude consumer
```

**代码事实：** 活动底盘路径见 `modules/imu/ins_task.c:119-169`。【已确认｜A级】

**原理解释：** 原始传感器、姿态滤波和应用坐标是三个独立步骤。当前底盘 FOLLOW 未使用 IMU，而使用 RS 相对角。

**改进建议：** 静止断能记录各层数据，先解释偏置和轴向，再研究 EKF 参数。

### 路径 G：自动事件 → 三次轨迹 → 关节目标

```text
AutoActionEventTransfer
→ AutoActionHandler
→ waypoint / FSM
→ compute_segment_time
→ CubicTrajectory_Init / Update
→ traj.pos[0..5]
→ target_arm_angle[0..5]
```

**代码事实：** 事件入口在 arm `robot_cmd.c:617-683`，轨迹在 `modules/fsm/auto_action.c:145-205` 与 `trajectory_planning.c:13-91`。【已确认｜A级】

**原理解释：** 状态机决定“去哪”，轨迹决定“随时间怎样去”，PID决定“电机怎样追踪”。三层不能互相代替。

**改进建议：** 最终审查确认 FSM size 越界是 P0；此路径只做离线教学，禁止实机运行。先阅读 [11-safety-analysis.md](11-safety-analysis.md) 的实机前最低门槛。

### 路径 H：设备 callback → daemon → 失联行为

```text
有效接收回调
→ DaemonReload
→ DaemonTask 每 10 tick 递减
→ counter == 0
→ lost_callback
→ 清输入 / 重启串口 / 标记离线
→ 下一控制周期决定是否 STOP
```

**代码事实：** 通用实现见 `modules/daemon/daemon.c`；遥控器、CANComm 和各电机的 lost callback 行为并不一致。【已确认｜A级】

**原理解释：** daemon 只负责“发现多久没更新并执行回调”，它不天然保证所有输出都变零。安全证据必须继续追到发送函数。

**改进建议：** 为每个设备填写“最后帧、检测时刻、STOP 时刻、最后非零命令”四个时间点，特别关注 LK 和板间失联。

## 3. 代码事实、原理和建议的练习方法

每读一个模块写三栏：

- **代码事实：** 路径、函数、结构体、条件、单位；必须能指到代码。
- **原理解释：** 为什么这样设计，以及输入输出的物理意义；不能冒充当前实现。
- **改进建议：** 可测试、可审查的下一步；不在阅读阶段直接修改安全参数。

例如：“FreeRTOS 周期为 1 ms”不是严格代码事实；准确写法是“任务每轮 `osDelay(1)`，tick 为 1 kHz，名义最小间隔 1 ms，实际周期还包含执行与调度延迟”。

## 4. 初学者阶段性成果

完成本路线后，学习者应能：从任一输入字段追到最终 CAN 字节；从反馈 CAN 字节追到 PID 测量值；解释两个控制板如何交换命令/状态；区分 STOP 软件状态和硬件急停；在不驱动机构的前提下判断超时、回绕、单位、符号和调度是否合理；明确哪些结论仍需要电机手册、遥控器手册、CAD/电气图和实机验证。
