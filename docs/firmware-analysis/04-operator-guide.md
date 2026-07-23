# 基于代码的操作逻辑草案

> 本文已通过第四阶段静态证据复核，但仍不是可直接上机执行的正式操作手册。所有可能引发动作的条目均标证据等级；未做实机验证，C/D 级内容只列为待确认事项。

> ⚠ **安全提示：** 下述按键、拨杆、自动动作和使能条件只用于解释代码。当前存在自动状态表越界、初始化完成判定不可靠和通信失联保留旧命令等风险；未经独立安全审查、硬件急停和断能台架验证，不得据此操作实机。

## 1. 输入设备与生效总条件

| 输入设备 | 接收入口 | 原始缓冲/字段 | 解析结果 | 首次影响控制的位置 | 状态 |
| --- | --- | --- | --- | --- | --- |
| WBUS/SBUS 遥控器 | 机械臂 USART3 的 `RemoteControlRxCallback()` | 25 B UART 缓冲 | `RC_ctrl_t` | `RobotCMDTask()` → `RemoteControlSet()` / 路径选择 / `EmergencyHandler()` | 活动【已确认｜A级】 |
| 图传键鼠 | 机械臂 USART1 的 `ImgTransRxCallback()` | A9 53 帧中偏移 10 的 11 B | `KeyMouseRaw`、`KeyMouseData`、`key_count` | SB 为 up 时 `MouseKeySet()` | 活动【已确认｜A级】 |
| 自定义控制器 | USART1 裁判协议 `0x0302`，30 B | `ControllerToRobot.data[30]` | 前 24 B 被解释为 6 个 `float`；末 6 B 为 keypad | F 切换打开后 `CustomControllerSet()` | 有条件活动【已确认｜A级】 |
| USB 上位机 | USB CDC `DecodeHost()` | 头 `0xAB` + 6 个 `float` | `Host_Recv_s.cmd_arm_angle[6]` | `HostControlSet()` | 接收活动，控制调用注释【已确认｜A级】 |
| 底盘本地遥控器 | 底盘 USART3 | `RC_ctrl_t` | 本地 `RemoteControlSet()` | `RobotCMDTask()` 本地分支 | 初始化存在，但控制分支注释【已确认｜A级】 |

主要依据：机械臂 `modules/remote/remote_control.c:9-171`、`modules/referee/rm_referee.c:34-253`、`modules/master_machine/host_communication.c:41-65`、`application/cmd/robot_cmd.c:884-921,956-1195`；底盘 `application/cmd/robot_cmd.c:101-132,606-619`。

`【已确认｜A级】` 机械臂动作的全局使能条件是遥控器在线且 `switch_SA` 满足 `switch_is_down(SA)`。否则 `EmergencyHandler()` 把机械臂与底盘命令模式置 STOP。定义和判断见 `remote_control.h:45-47`、`robot_cmd.c:543-612`。这并不能证明真实设备具备独立硬件急停。

## 2. 遥控器协议与数值范围

WBUS 字段映射由解析代码直接给出：

| 代码通道 | WBUS 原始字段 | 解析变量 | 数值处理 | 可信度 |
| --- | --- | --- | --- | --- |
| J1 | 11-bit J1 | `rocker_r_` | 减 1024 | 【已确认｜A级】 |
| J2 | 11-bit J2 | `rocker_r1` | 减 1024 | 【已确认｜A级】 |
| J3 | 11-bit J3 | `rocker_l1` | 减 1024 | 【已确认｜A级】 |
| J4 | 11-bit J4 | `rocker_l_` | 减 1024 | 【已确认｜A级】 |
| SA—SD | 各 11-bit 开关/模拟字段 | `switch_SA`—`switch_SD` | 减 1024；down `>10`，mid `[-10,10]`，up `<-10` | 【已确认｜A级】 |

依据：`remote_control.c:97-116`、`remote_control.h:31-47,100-109`。代码声明原始范围 364—1684、中值 1024，即解析后标称约 -660—660；`RectifyRCjoystick()` 将前 5 个通道绝对值大于 671 的异常值清零，但没有通用摇杆死区和低通滤波（`remote_control.c:23-29`）。接收注释约 70 Hz；守护计数 10、守护任务 10 ms，约 100 ms 无帧判离线并清零整个 `RC_ctrl_t`（`remote_control.c:141-162`）。

物理“左/右摇杆”文字只来自结构体注释，尚无遥控器型号或面板资料交叉验证；下文优先使用 J1—J4、SA—SD。

## 3. 遥控器操作映射

以下是代码效果，不是上机操作指令。

| 路径条件 | 操作项 | 原始/解析变量 | 控制对象 | 实际代码作用 | 可信度 |
| --- | --- | --- | --- | --- | --- |
| SA down | 总使能 | `switch_SA` | 全系统 | 允许 `ROBOT_READY` 与机械臂 ENABLE；否则两模式 STOP | 【已确认｜A级】 |
| SB down + SC down | J4/J3/J1 | `rocker_l_` / `rocker_l1` / `rocker_r_` | 底盘 | `vx=10*J4`、`vy=10*J3`、`wz=1.5*J1` | 【已确认｜A级】 |
| SB down + SC down | J2 | `rocker_r1` | 底盘姿态通道 4 | 每 5 ms 增加 `0.00003*J2` | 【已确认｜A级】 |
| SB down + SC mid | J2/J3 | `rocker_r1` / `rocker_l1` | 底盘 4 个姿态通道 | 通道 0/1 反向成对变化，2/3 反向成对变化 | 【已确认｜A级】 |
| SB down + SC mid | J1/J4 | `rocker_r_` / `rocker_l_` | 底盘 | `vx=5*J1`、`vy=5*J4` | 【已确认｜A级】 |
| SB down + SC mid | SD down | `switch_SD` | 履带 | 置 `START_TRACK`，底盘转为 +4000 速度目标 | 【已确认｜A级】 |
| SB down + SC up | SD up / 非 up | `switch_SD` | 底盘模式 | SD up 选择 FOLLOW，否则 ROTATE | 【已确认｜A级】 |
| SB down + SC up | J4/J3/J1 | 相应通道 | 底盘 | `vx=5*J4`、`vy=5*J3`；姿态通道 4 减 `0.00003*J1` | 【已确认｜A级】 |
| SB mid + SC down | J4/J3/J2 | 相应通道 | 机械臂通道 0/1/2 | 分别按 `-1e-5`、`-1e-5`、`-2e-5` 累加目标 | 【已确认｜A级】 |
| SB mid + SC mid | J2/J1/J3 | 相应通道 | 机械臂通道 3/4/5 | 分别按 `2e-5` 累加目标 | 【已确认｜A级】 |
| SB mid + SC mid | J4 | `rocker_l_` | 机械臂通道 6 | 直接写目标；该通道在电机配置中是速度环，不是位置角 | 【已确认｜A级】 |
| SB mid + SC mid | SD down/up | `switch_SD` | 末端夹持自动流程 | 根据 `user_state` 触发开/合动作 | 【较可信｜B级】 |
| SB mid + SC up | J3/J4 | 相应通道 | 图传舵机 | pitch `+=1e-6*J3`，yaw `-=1e-6*J4`；该分支 IK 代码已注释 | 【已确认｜A级】 |
| SB up | 路径选择 | `switch_SB` | 键鼠/自动动作 | 不调用 `RemoteControlSet()`，改调用 `MouseKeySet()` 与 `AutoActionHandler()` | 【已确认｜A级】 |

依据：机械臂 `robot_cmd.c:271-405,896-910`。代码在 `RemoteControlSet()` 开头先把底盘模式置 ENABLE；FOLLOW/ROTATE 只在特定分支覆盖（`robot_cmd.c:271-385`）。

## 4. 键鼠映射

图传解析把“无修饰键、仅 Ctrl、仅 Shift”分到三个槽；Ctrl 与 Shift 同时按下时三个槽均不保存该帧组合。上升沿计数索引顺序由 `key_mouse_t` 位域定义为 W/S/D/A/Shift/Ctrl/Q/E/R/F/G/Z/X/C/V/B。依据：`rm_referee.c:34-70`、`referee_protocol.h:407-435`。

| 操作项 | 生效条件 | 控制对象 | 代码效果 | 可信度 |
| --- | --- | --- | --- | --- |
| W/S | SB up、无修饰 | 底盘 `vy` | 按住逐周期加/减；上限 +6000、下限 -5000；松开逐步回零 | 【已确认｜A级】 |
| A/D | SB up、无修饰 | 底盘 `vx` | 按住逐周期加/减；上限 +3500、下限 -2500；松开逐步回零 | 【已确认｜A级】 |
| Shift+W/S/A/D | SB up、仅 Shift | 底盘速度 | 直接组合为 ±1000 的低速目标 | 【已确认｜A级】 |
| Mouse X | SB up | 底盘姿态通道 4 | 每周期 `target[4] -= 0.00008*mouse_x` | 【已确认｜A级】 |
| B | SB up、无修饰 | 履带 | 按住置 `START_TRACK_INVERSE`，底盘目标 -4000 | 【已确认｜A级】 |
| C | SB up、无修饰 | 两个 DM4310 | 按住置 `DM4310ReCmd` | 【已确认｜A级】 |
| F（上升沿奇偶） | SB up | 自定义控制器模式 | 奇数状态调用 `CustomControllerSet()`；自动动作事件非零时取消该模式 | 【已确认｜A级】 |
| G（上升沿） | SB up | 夹持流程 | 切换 `grabber_action`；泵控制代码本身已注释 | 【较可信｜B级】 |
| Shift+B | SB up | 机械臂通道 6 | 目标速度 400，约 30 个命令周期后清零并转入夹持状态 | 【已确认｜A级】 |
| V（上升沿奇偶） | SB up | 图传舵机 | 开启时调用 `ImgControlSet()` 跟随机械臂反馈；关闭时鼠标 Y 调 pitch | 【已确认｜A级】 |
| Z（上升沿奇偶） | SB up | 一组底盘姿态通道 | 运行展开/收回计数逻辑；条件中存在近乎恒真的 `x<0.65 || x>-0.65` | 【已确认｜A级】 |
| Q/E/R 与 Shift+Q/Shift+E/X | SB up 且自动事件空闲/忙碌条件满足 | 自动动作 | 按固定 else-if 优先链触发存放、抓取、复位或 reformat | 【已确认｜A级】 |

主要依据：机械臂 `robot_cmd.c:617-683,956-1195`。键盘动作的物理安全性、真实运动方向和是否会与场地机构碰撞均未确认，因此不能把本表直接用作训练步骤。

## 5. 自定义控制器与自动动作

`CustomControllerSet()` 从 `ControllerToRobot.data` 前 24 B 复制 6 个浮点目标，叠加偏置后进行 `0.05*新值 + 0.95*旧值` 低通，并初始化/更新六自由度三次轨迹。其活动入口是键鼠 F 切换，顶层原本的无条件调用已注释。依据：机械臂 `robot_cmd.c:413-499,921,1151-1170`。

自动动作事件按 `AutoActionEventTransfer()` 的 `if/else if` 顺序具有优先级；相同周期只会选中第一个条件。自动轨迹只覆盖 `target_arm_angle[0..5]`，第 6 通道由夹持逻辑单独使用。依据：`robot_cmd.c:617-700`、`trajectory_planning.h:13-32`。

`【已确认｜A级】` 当前自动动作状态表存在 `fsm.size=47` 越界问题。在修复并完成台架验证前，不应把任何自动抓取、存放、复位或爬台动作写为可执行操作说明。

## 6. 模式、停止与恢复

| 模式/状态 | 进入条件 | 退出条件 | 目标来源 | 输出行为 | 优先级/备注 |
| --- | --- | --- | --- | --- | --- |
| `ROBOT_INIT` | 上电静态初值；从 DIE 恢复 | `MotorCmdDataInit()` 判定完成 | 初始化反馈 | 机械臂/底盘模式 STOP | 判定实现有未初始化变量风险【已确认｜A级】 |
| `ROBOT_READY` | 遥控器在线、SA down，且不在 INIT/DIE | 遥控器离线/SA 非 down，或机械臂 DIE | 当前输入路径 | 机械臂 ENABLE；底盘由输入函数设定 | 活动正常状态【已确认｜A级】 |
| `ROBOT_STOP` | 遥控器离线或 SA 非 down | 遥控器在线且 SA down | STOP 时把目标复制为当前反馈 | 机械臂和板间底盘模式 STOP | 软件急停，不等于硬件断能【已确认｜A级】 |
| `ROBOT_DIE` | `gimbal_state == GIMBAL_DIE` | `gimbal_state == GIMBAL_ALIVE` 后转 INIT | 无 | 两模式 STOP | 离线判据不覆盖全部电机【已确认｜A级】 |
| 底盘 ENABLE | 遥控器部分分支或板间字段 | FOLLOW/ROTATE/STOP 覆盖 | `vx/vy/wz` | 直接四轮解算 | 【已确认｜A级】 |
| 底盘 FOLLOW | SB down、SC up、SD up；键鼠路径也固定设 FOLLOW | 其他分支/STOP | 平移目标 + 姿态角 PID | 旋转平移坐标并令相对角趋零 | 【已确认｜A级】 |
| 底盘 ROTATE | SB down、SC up、SD 非 up | 其他分支/STOP | 平移目标 + 常数 `wz=4000` | 持续旋转 | 【已确认｜A级】 |

STOP 时机械臂命令目标被设置为反馈值，电机对象随后进入 Stop；底盘 STOP 目标也被机械臂板复制为最近一次底盘反馈。但底盘板断开板间通信后不会自动进入 STOP，这是当前最重要的停止链缺口。

模式切换没有统一清空全部 PID 积分，也没有统一把所有目标通过斜坡接续；不同电机驱动的 Stop 行为不同。故“切换必然无冲击”无法由代码确认。【代码中未确认｜D级】

## 7. 未确认的实机操作

下列内容不得作为正式指令：

- SA/SB/SC/SD 在真实遥控器面板上的物理方向、拨杆标签及失联姿态。【代码中未确认｜D级】
- J1—J4 正方向与机械臂/底盘真实运动方向，尤其电机安装反向后的整机效果。【代码中未确认｜D级】
- 自动动作 waypoint 是否适配当前机械装配、零点和场地对象。【代码中未确认｜D级】
- FOLLOW/ROTATE 模式下底盘真实旋转方向、速度与碰撞空间。【代码中未确认｜D级】
- C、G、Z、Shift+B 等组合键在当前硬件版本上的安全结果。【代码中未确认｜D级】
- 软件 STOP 是否能在所有电机驱动器故障、CAN 堵塞或 MCU 故障情况下真正断能。【代码中未确认｜D级】

## 8. 最终操作手文档验收

**代码事实：** 当前自动动作状态表大小与实际表项数不一致；两板初始化完成函数含未初始化局部计数；底盘板间失联后仍可能保留旧命令；底盘未初始化分支仍使能四轮。上述问题分别见 `modules/fsm/auto_action.c`、两板 `MotorCmdDataInit()`、`modules/can_comm/can_comm.c` 和底盘 `ChassisTask()`。【已确认｜A级】

**原理解释：** 软件 STOP 是 MCU 逻辑状态，不是接触器、安全继电器或电源级急停。调试器断点、任务卡死或 CAN 发送堵塞都可能改变其生效时序。

**改进建议：** 本文所有映射只用于断能观察和离线教学；尤其禁止运行自动动作。安全调试顺序见 [10-parameters-and-tuning.md](10-parameters-and-tuning.md)，故障处置见 [12-troubleshooting.md](12-troubleshooting.md)，最终安全矩阵见 [11-safety-analysis.md](11-safety-analysis.md)。

| 验收类别 | 内容 | 当前结论 |
| --- | --- | --- |
| 1. 已由代码确认 | USART/CAN 接收入口、SA—SD/J1—J4 数值条件、模式变量、目标写入、软件 STOP 分支 | 可作为代码理解和断能调试依据【已确认｜A级】 |
| 2. 需要接线图确认 | 两板 CAN 外设到物理线束、硬件急停、驱动器 enable、电机 ID 与接头 | 缺 PCB/电气资料【代码中未确认｜D级】 |
| 3. 需要遥控器型号确认 | SA—SD/J1—J4 对应面板位置、物理 up/down、失联姿态 | 不能把代码字段翻译为正式面板指令【代码中未确认｜D级】 |
| 4. 需要实机验证 | 电机/关节方向、速度手感、STOP 时延、模式切换冲击、限位、自动 waypoint | 必须由有经验人员在独立断能和限流条件下验证【代码中未确认｜D级】 |
| 5. 当前不能作为正式操作依据 | 自动抓取/存放/复位/爬台、任何宣称“急停必然覆盖全部故障”的表述 | 5 个 P0 尚未修复；明确禁止据此操作实机 |

最终结论：文档可用于学习代码和准备验证矩阵，但没有通过实机安全验证，不能作为正式操作手册。
