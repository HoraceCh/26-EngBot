# 控制算法与关键数据字典

> 本文只解释当前仓库确实存在的算法。机械臂运动学代码目前未接入活动控制链，不能把它当成整机现状。行号以当前工作树为准；两工程同名算法文件大体一致时，以明确列出的工程为证据。

## 1. 算法在控制链中的位置

```mermaid
flowchart LR
    IN["遥控器 / 键鼠 / 自动事件"] --> TARGET["关节角或底盘速度目标"]
    TARGET --> TRAJ["三次轨迹 / 限幅 / 一阶滤波"]
    TRAJ --> KIN["底盘轮速解算\n机械臂 IK 当前未接入"]
    KIN --> OUTER["位置 PID（可选）"]
    OUTER --> INNER["速度 PID"]
    INNER --> CMD["电流或力矩命令"]
    CMD --> MOTOR["电机与机械结构"]
    MOTOR --> FEEDBACK["编码器 / 速度 / IMU"]
    FEEDBACK --> OUTER
```

**代码事实：** 活动机械臂通道中，`joint_0..5` 使用位置—速度串级，`joint_6` 使用速度单环；底盘四轮和两条履带使用速度单环，另外的关节电机使用位置—速度串级。电机驱动的闭环调用分别位于 `Engineer_arm/modules/motor/RSmotor/rsmotor.c:244-335`、`DMmotor/dmmotor.c:289-363`、`LKmotor/LKmotor.c:107-192` 和底盘 `DJImotor/dji_motor.c:281-342`。【已确认｜A级】

**原理解释：** 上游算法只生成“希望系统到哪里/多快”，PID 把目标与反馈的误差变成驱动器命令。位置环输出通常作为速度环目标；本仓库没有活动的软件电流内环，最内层电流/力矩闭环由电机驱动器或电机自身协议承担。

**改进建议：** 维护时先画清每个通道的单位链，再调参数。不要把 RS/DM 的 N·m 命令、LK/DJI 的协议电流值和关节实际力矩当成同一物理量。

## 2. PID 实现

### 2.1 数学与代码对应

`PIDCalculate()` 先读取 DWT 实测间隔 `dt`，计算误差

$$e_k=r_k-y_k$$

再计算

$$P_k=K_p e_k$$

活动电机实例启用了梯形积分：

$$I_k=\operatorname{clamp}\left(I_{k-1}+K_i\frac{e_k+e_{k-1}}{2}\Delta t,\,-I_{max},I_{max}\right)$$

并采用“对测量微分”以降低目标阶跃造成的微分冲击：

$$D_k=K_d\frac{y_{k-1}-y_k}{\Delta t}$$

最终

$$u_k=\operatorname{clamp}(P_k+I_k+D_k,\,-u_{max},u_{max})$$

**代码事实：** 公式对应 `Engineer_arm/modules/algorithm/controller.c:38-73,130-198`。`f_Integral_Limit()` 在输出已饱和且误差继续推动同方向饱和时阻止新的积分，并限制 `Iout`；`PIDCalculate()` 最后限制 `Output`。活动电机配置普遍选择 `PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement`。【已确认｜A级】

**原理解释：** 梯形积分比矩形积分更平滑；对测量微分不会因目标突然改变而产生同样大的微分踢；积分限幅与条件积分用于减轻执行器饱和后的积分累积。

**改进建议：** `dt` 虽来自实测，但代码没有显式拒绝极小或异常 `dt`。高 `Kd` 通道应记录 `dt` 和 `Dout`。此外，死区分支把 `Output`、`ITerm` 置零，却没有清除已累计的 `Iout`（`controller.c:158-164`），注释与实际状态不完全一致。

### 2.2 31 个活动 PID 环

“活动”指所属闭环位在实例配置中启用，或在活动 INS 路径中被调用。驱动结构体里虽总会初始化 `current_PID`，但未被闭环位使用的零配置实例不计入。

| PID 实例 | 控制对象 | 环 | 目标 / 反馈 | 输出 | 参数 `Kp,Ki,Kd` | 参数位置 | 输出限幅 |
| --- | --- | --- | --- | --- | --- | --- | ---: |
| arm `joint_0.angle` | RS `arm_06_1` | 外环 | rad / rad | rad/s 目标 | 28, 8, 0.5 | `Engineer_arm/application/gimbal/gimbal.c:28-49` | 20 |
| arm `joint_0.speed` | 同上 | 内环 | rad/s / rad/s | N·m 命令 | 0.95, 0, 0 | 同上 | 10 |
| arm `joint_1.angle` | RS `arm_04_2` | 外环 | rad / rad | rad/s 目标 | 200, 300, -0.8 | `gimbal.c:69-91` | 30 |
| arm `joint_1.speed` | 同上 | 内环 | rad/s / rad/s | N·m 命令 | 0.32, 0, 0 | 同上 | 10 |
| arm `joint_2.angle` | RS `arm_06_3` | 外环 | rad / rad | rad/s 目标 | 50, 3.5, 0.8 | `gimbal.c:108-132` | 20 |
| arm `joint_2.speed` | 同上 | 内环 | rad/s / rad/s | N·m 命令 | 0.8, 0, 0 | 同上 | 10 |
| arm `joint_3.angle` | DM `arm_4310_4` | 外环 | rad / rad | rad/s 目标 | 30, 0, 0 | `gimbal.c:146-173` | 8 |
| arm `joint_3.speed` | 同上 | 内环 | rad/s / rad/s | MIT torque | 1, 0, 0 | 同上 | 10 |
| arm `joint_4.angle` | DM `arm_4310_5` | 外环 | rad / rad | rad/s 目标 | 30, 0, 0 | `gimbal.c:180-207` | 5 |
| arm `joint_4.speed` | 同上 | 内环 | rad/s / rad/s | MIT torque | 1, 0, 0 | 同上 | 5 |
| arm `joint_5.angle` | LK `arm_4005_6` | 外环 | rad / rad | rad/s 目标 | 400, 0, 2 | `gimbal.c:220-247` | 500 |
| arm `joint_5.speed` | 同上 | 内环 | rad/s / rad/s | 电流协议值 | 5, 0, 0 | 同上 | 1000 |
| arm `joint_6.speed` | LK `arm_4005_7` | 单环 | rad/s / rad/s | 电流协议值 | 1.5, 0, 0 | `gimbal.c:258-286` | 500 |
| chassis `drive[0..3].speed`（4 环） | DJI 四轮 | 单环 | rpm-like / rpm | 电流协议值 | 15, 0, 0 | `Engineer_chassis/application/chassis/chassis.c:96-137` | 15000 |
| chassis `track[0..1].speed`（2 环） | DJI M3508 两履带 | 单环 | rpm-like / rpm | DJI 电流协议值 | 3.5, 0, 0 | `chassis.c:114-135` | 16000 |
| chassis `arm_04_0.angle` | RS | 外环 | rad / rad | rad/s 目标 | 40, 0, 0 | `chassis.c:178-203` | 30 |
| chassis `arm_04_0.speed` | 同上 | 内环 | rad/s / rad/s | N·m 命令 | 0.4, 0, 0 | 同上 | 20 |
| chassis `rear_leg[0..1].angle`（2 环） | RS 两后腿 | 外环 | rad / rad | rad/s 目标 | 45, 0, 0 | `chassis.c:206-238` | 50 |
| chassis `rear_leg[0..1].speed`（2 环） | 同上 | 内环 | rad/s / rad/s | N·m 命令 | 0.6, 0, 0 | 同上 | 30 |
| chassis `rocker[0..1].angle`（2 环） | DM 两摇臂 | 外环 | rad / rad | rad/s 目标 | 50, 0, 0 | `chassis.c:214-253` | 330 |
| chassis `rocker[0..1].speed`（2 环） | 同上 | 内环 | rad/s / rad/s | MIT torque | 2.5, 0, 0 | 同上 | 20 |
| `chassis_wz_PID` | 底盘跟随角 | 单环 | 0 / RS 相对角 rad | `wz` 代码量 | -600, 0, -30 | `chassis.c:255-265,412` | 9000 |
| `TempCtrl` | BMI088 加热 | 单环 | °C / °C | PWM compare | 1000, 20, 0 | `Engineer_chassis/modules/imu/ins_task.c:42-51,103-110` | 2000 |

合计：机械臂 13 环，底盘 18 环，31 环。【已确认｜A级】表中 4/2/2 等括号项按实例数计数。

### 2.3 PID 的实现边界和风险

| 检查项 | 当前实现 | 判断 |
| --- | --- | --- |
| 积分限幅 / 抗饱和 | 有 `f_Integral_Limit()` 和 `IntegralLimit` | 【已确认｜A级】 |
| 输出限幅 | 每环 `MaxOut` | 【已确认｜A级】 |
| 微分 | 活动电机环对测量微分 | 【已确认｜A级】 |
| 微分/输出滤波 | 框架存在，活动实例未配置 | 【已确认｜A级】 |
| 积分分离 | 未发现该策略 | 【代码中未确认｜D级】 |
| 时间步长 | DWT 实测秒数，不是写死常数 | 【已确认｜A级】 |
| 重置逻辑 | DJI 停止清角度/速度积分；RS 只清角度积分；DM/LK 不统一 | 【已确认｜A级】 |
| 并发安全 | `f_Integral_Limit()` 使用函数内 `static temp_Output/temp_Iout` | 【已确认｜A级】P2 |

**原理解释：** 函数内 `static` 临时量由所有 PID 实例共享。底盘 `INSTask` 优先级高于普通电机任务；若它在另一个 PID 正计算时抢占，两个实例可能互相覆盖临时量。

**改进建议：** 最终审查将该跨实例临时量风险定为 P2；更严重的共同实现问题是浮点 PID 5 处调用整数 `abs()`（P1）。修复前应以主机单测覆盖小误差、积分限幅、饱和、`dt=0` 和非有限输入；本文未修改控制代码。模式切换的积分状态也应统一定义和测试。

## 3. 三次轨迹规划

对六个通道分别使用三次多项式：

$$q(t)=a_0+a_1t+a_2t^2+a_3t^3$$

其中 $q(0)=q_0$、$\dot q(0)=v_0$、$q(T)=q_f$、$\dot q(T)=v_f=0$，所以：

$$a_0=q_0,\quad a_1=v_0$$

$$a_2=\frac{3(q_f-q_0)}{T^2}-\frac{2v_0+v_f}{T}$$

$$a_3=-\frac{2(q_f-q_0)}{T^3}+\frac{v_0+v_f}{T^2}$$

**代码事实：** `compute_segment_time()` 取六关节最大角差；差值小于 0.8 rad 时名义最大速度 0.5 rad/s，否则 1.8 rad/s，并额外加 0.1 s。`CubicTrajectory_Update()` 用 DWT 实际 `dt` 累加时间并同时计算 `pos[6]`、`vel[6]`（`Engineer_arm/modules/algorithm/trajectory_planning.c:13-91`）。自动动作从 `auto_action.c:145-205` 调用；活动路径只把位置送入电机目标，`vel` 没有用作前馈。【已确认｜A级】

**原理解释：** 三次曲线保证段首、段末的位置和速度条件连续，但没有显式限制加速度/加加速度；“按最大关节差选统一 T”让所有关节同时结束。

**改进建议：** 实机调试前离线画出位置、速度和加速度，并检查相邻 waypoint 的速度继承。状态表 `fsm.size=47` 与机械臂实际 24 个状态项不一致，可能越界，属于 P0，不能把自动动作作为初学者实机实验。

## 4. 机械臂正逆运动学：存在但未接入

**代码事实：** `Engineer_arm/modules/algorithm/kinematics.c` 定义 `ForwardKinematics()` 和 `InverseKinematics()`，机械参数 `a2=0.33`、`d4=0.39`、`d6=0` 见 `kinematics.h:17-25`。当前命令路径中的调用被注释，活动控制仍直接写 `target_arm_angle[]`。【已确认｜A级】

平面两连杆部分使用

$$r=x^2+y^2$$

$$\theta_3=\pm\arccos\frac{r-a_2^2-d_4^2}{2a_2d_4}$$

$$\sigma=\operatorname{atan2}(y,x),\qquad
\phi=\arccos\frac{r+a_2^2-d_4^2}{2a_2\sqrt r}$$

$$\theta_2=\sigma\pm\phi$$

后三级再从 ZYX 欧拉旋转矩阵解角。正解的平面位置为

$$x=a_2\cos\theta_2+d_4\cos(\theta_2+\theta_3),\quad
y=a_2\sin\theta_2+d_4\sin(\theta_2+\theta_3)$$

**原理解释：** 这是“直线/两转动关节 + 姿态腕部”的几何解线索，不足以确认真实坐标轴、零位和物理关节命名。长度值看似以 m 为尺度，但代码没有明确单位声明，文档不将其提升为 A 级事实。

**改进建议：** `acos` 前没有把输入钳制到 `[-1,1]`，$r=0$ 和 $\sin\theta_5=0$ 也形成奇异条件；代码只在结果为 NaN 后保持旧目标。接入前必须用 CAD/DH 参数、零点、方向和工作空间测试确认。【较可信｜B级】

## 5. 底盘运动学与坐标旋转

### 5.1 四轮解算

**代码事实：** `Engineer_chassis/application/chassis/chassis.c:288-294` 的 `MecanumCalculate()` 计算：

$$v_{LF}=v_x+v_y+\omega_z L_{LF}$$
$$v_{RF}=v_x-v_y+\omega_z L_{RF}$$
$$v_{LB}=-v_x+v_y+\omega_z L_{LB}$$
$$v_{RB}=-v_x-v_y+\omega_z L_{RB}$$

其中四个 $L$ 由半轴距、半轮距、中心偏置再乘 `DEGREE_2_RAD` 得到。函数名和符号关系支持“麦克纳姆式四轮解算”的软件意图，但仓库没有机械装配图，实际轮型仍为 B 级。

`WHEEL_BASE=400`、`TRACK_WIDTH=377`、`RADIUS_WHEEL=60` 位于 `Engineer_chassis/application/robot_def.h:43-47`；轮半径只用于未进入当前公式的周长宏，源代码没有明确这三个量的物理单位。【已确认｜A级】

### 5.2 跟随坐标系

`RotateMoveResolve()` 用底盘板 RS 电机 `arm_04_0->measure.angle` 对 $(v_x,v_y)$ 做二维旋转：

$$\begin{bmatrix}v_x'\\v_y'\end{bmatrix}=
\begin{bmatrix}\cos\theta&\sin\theta\\-\sin\theta&\cos\theta\end{bmatrix}
\begin{bmatrix}v_x\\v_y\end{bmatrix}$$

FOLLOW 模式用 `PIDCalculate(&chassis_wz_PID, angle, 0)` 令相对角趋于零；ROTATE 模式把 `chassis_wz` 固定为 4000。板间传来的 `theta_z` 当前被注释而未参与控制（`chassis.c:336-349,392-420`）。【已确认｜A级】

**原理解释：** 旋转平移向量是在两个水平坐标系之间变换；它不等于用 IMU 建立世界坐标系。当前底盘姿态数据未进入活动平移解算。

**改进建议：** 用抬轮台架逐轴验证四个符号和电机反向标志；没有机械资料时不要把正方向写成正式驾驶说明。

## 6. 滤波、姿态估计和功率缩放

### 6.1 一阶低通

自定义控制器目标使用

$$y_k=0.05x_k+0.95y_{k-1}$$

见 `Engineer_arm/application/cmd/robot_cmd.c` 的自定义控制器设置路径。【已确认｜A级】

INS 去重力后的运动加速度使用随 `dt` 变化的一阶低通：

$$a_{f,k}=a_k\frac{\Delta t}{\tau+\Delta t}+a_{f,k-1}\frac{\tau}{\tau+\Delta t},\quad \tau=0.0085$$

见 `Engineer_chassis/modules/imu/ins_task.c:113,154-163`。【已确认｜A级】

DJI/LK 反馈滤波宏采用 `filtered=(1-coef)*old+coef*new`；因此 0.85/0.9 表示更偏重新样本，并不是“系数越大越平滑”。应先核对公式再调宏值。

### 6.2 四元数 EKF

**代码事实：** 活动底盘 `INSTask` 依次执行 `BMI088_Read()`、坐标旋转、`IMU_QuaternionEKF_Update()`，再把 yaw/pitch/roll 与累计 yaw 从 degree 转成 rad 发布。初始化参数为 `(10, 0.001, 1000000, 1, 0)`（`Engineer_chassis/modules/imu/ins_task.c:80-169`；EKF 入口 `modules/algorithm/QuaternionEKF.c:50-197`）。【已确认｜A级】

**原理解释：** 陀螺仪积分负责短时姿态传播，加速度重力方向帮助修正倾斜和陀螺零偏。本文不逐行展开矩阵库和卡尔曼通用实现。

**改进建议：** 当前活动底盘跟随不使用 IMU；在把姿态引入控制前，要先确认安装旋转矩阵、静止噪声、温漂和单位。`INS_Init()` 会在 BMI088 初始化失败时持续忙等待，也应纳入启动故障诊断。

### 6.3 功率控制

**代码事实：** `Engineer_chassis/modules/power_control/power_control.c:31-85` 用二次模型预测功率 $P(k)=ak^2+bk+c$；预测超过 `chassis_power_set=105` 时解方程获得 $k\in[0,1]$ 并缩放轮速目标。【已确认｜A级】

**原理解释：** 这是对目标整体缩放，不是每台电机独立限流，也没有发现裁判系统实时功率/缓冲能量进入活动计算。

**改进建议：** `LimitChassisOutput()` 外层循环四次调用一个内部已遍历四轮并缩放全部目标的函数，可能在一次周期重复衰减；且模型读取的是 PID 当前状态。最终审查将其定为 P2，应通过离线回放核对调用次数和时序。

## 7. 未发现的活动算法

| 算法 | 结论 | 可信度 |
| --- | --- | --- |
| 重力补偿 | 活动链未发现 | 【代码中未确认｜D级】 |
| 摩擦补偿 | 活动链未发现 | 【代码中未确认｜D级】 |
| 速度/力矩前馈 | 驱动框架有字段，活动实例未配置；轨迹速度未下送 | 【已确认｜A级】 |
| 活动机械臂 IK/FK | 实现存在，调用被注释 | 【已确认｜A级】 |
| 世界坐标底盘控制 | IMU 姿态未进入活动底盘速度解算 | 【已确认｜A级】 |
| S 曲线/五次轨迹 | 未发现活动实现 | 【代码中未确认｜D级】 |

## 8. 关键数据字典

| 名称 | 类型 | 定义位置 | 单位 | 初始化者 | 更新者 | 使用者 | 生命周期 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `RC_ctrl_t` | 遥控器结构 | 两工程 `modules/remote/remote_control.h:69-92` | 通道原始计数/按键位 | `RemoteControlInit()` | UART Rx 回调 | `RobotCMDTask()` | 静态实例，持续运行 |
| `ImgTrans_info_t` | 图传/键鼠结构 | `modules/referee/rm_referee.h` | 鼠标计数、键位、协议量 | `ImgTransInit()` | USART1 回调 | 机械臂命令任务 | 静态实例 |
| `Gimbal_Ctrl_Cmd_s` | 机械臂控制命令 | `Engineer_arm/application/robot_def.h` | `target_arm_angle[0..5]` rad；索引6速度 | `RobotCMDInit()` | `RobotCMDTask()` | `GimbalTask()` | 消息中心主题 |
| `Gimbal_Upload_Data_s` | 机械臂反馈 | 同上 | 七路角度 rad、状态 | `GimbalInit()` | `GimbalTask()` | `RobotCMDTask()` | 消息中心主题 |
| `Chassis_Ctrl_Cmd_s` | 底盘内部命令 | `Engineer_chassis/application/robot_def.h` | 速度/角度混合，部分单位未标定 | `ChassisInit()` | 底盘 `RobotCMDTask()` | `ChassisTask()` | 消息中心主题 |
| `Chassis_Upload_Data_s` | 底盘内部反馈 | 同上 | 角度 rad、状态 | `ChassisInit()` | `ChassisTask()` | 命令任务 | 消息中心主题 |
| `Master_Cmd_Data_s` | 机械臂→底盘板间帧 | 两工程 `application/robot_def.h` | 目标数组、事件位，单位见协议文档 | `RobotCMDInit()` | 机械臂命令任务 | 底盘命令任务 | CANComm 缓冲 |
| `Slave_Upload_Data_s` | 底盘→机械臂板间帧 | 同上 | 反馈数组、事件位 | `RobotCMDInit()` | 底盘命令任务 | 机械臂命令任务 | CANComm 缓冲 |
| `PIDInstance` | PID 状态 | `modules/algorithm/controller.h:68-111` | 随环路而变 | 各设备 Init | `PIDCalculate()` | 电机/INS 任务 | 静态电机实例 |
| `DJIMotorMeasure_t` | DJI 反馈 | `modules/motor/DJImotor/dji_motor.h` | ecd、rpm、原始电流、°C | `DJIMotorInit()` | CAN 回调 | MotorTask/应用 | 静态实例 |
| `RSMotorMeasure_t` | RS 反馈 | `modules/motor/RSmotor/rsmotor.h` | rad、rad/s、N·m、°C | `RSMotorInit()` | CAN 回调 | PID/应用 | 静态实例 |
| `DMMotorMeasure_t` | DM 反馈 | `modules/motor/DMmotor/dmmotor.h` | rad、rad/s、N·m、°C、错误码 | `DMMotorInit()` | CAN 回调 | DMTask/应用 | 静态实例 |
| `LKMotorMeasure_t` | LK 反馈 | `modules/motor/LKmotor/` 头文件 | 编码器、rad、rad/s、协议电流、°C | `LKMotorInit()` | CAN 回调 | MotorTask/应用 | 静态实例 |
| `attitude_t` | INS 输出 | `modules/imu/ins_task.h:56-68` | rad、rad/s 等 | `INS_Init()` | `INS_Task()` | 底盘应用/图传 | 静态单例 |
| `AutoActionInstance` | 自动状态机 | `modules/fsm/auto_action.h` | 状态/时间/目标 | `AutoActionInit()` | 命令任务 | 自动动作处理 | 静态实例 |
| `CubicTrajectory` | 六轴轨迹状态 | `modules/algorithm/trajectory_planning.h:13-32` | rad、rad/s、s | `CubicTrajectory_Init()` | `CubicTrajectory_Update()` | 自动动作/命令 | 动作段内持续 |
| `DaemonInstance` | 在线监测 | `modules/daemon/daemon.h` | 10 ms 计数 | 各模块 Init | ISR reload / DaemonTask decrement | 丢失/重连回调 | 静态注册表 |
| `CANCommInstance` | 板间多帧协议 | `modules/can_comm/can_comm.h` | byte/帧计数 | `CANCommInit()` | CAN ISR / 命令任务 | 两板命令任务 | 静态实例 |
| `gimbal_mode` / `chassis_mode` | 控制模式枚举 | 两工程 `application/robot_def.h:96-109` | 枚举 | 命令初始化 | `EmergencyHandler()` 等 | 业务控制任务 | 全运行期 |
| `event` | 板间事件位图 | `application/robot_def.h:237-245` 附近 | bitmask | 命令初始化 | 两板事件处理 | 两板状态机 | 每帧覆盖/握手 |

**改进建议：** 上表多为 ISR 与任务共享的静态数据。最终审查确认消息中心、CANComm、遥控器与电机测量没有统一快照/互斥机制；`volatile` 只能阻止部分编译器优化，不能保证结构体多字段一致。建议逐个采用短临界区、双缓冲或版本号方案，详见 [11-safety-analysis.md](11-safety-analysis.md)。
