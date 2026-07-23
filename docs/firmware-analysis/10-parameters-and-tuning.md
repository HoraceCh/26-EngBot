# 参数、编译、烧录与安全调试

> 本文是代码审阅与维护指南，不是上电操作授权。凡涉及动力、电机使能、IMU 加热、烧录到实机或机构运动，必须由熟悉该机器人、具备硬件急停和限能条件的人员执行。

## 1. 参数如何阅读

**代码事实：** 参数分散在 `application/robot_def.h`、机械臂 `application/gimbal/gimbal.c`、底盘 `application/chassis/chassis.c`、`modules/algorithm/trajectory_planning.c`、`modules/imu/ins_task.c` 和各通信/daemon 初始化中。没有集中参数服务器，也没有版本化参数文件。【已确认｜A级】

**原理解释：** 参数必须连同“单位、调用周期、目标和反馈换算、输出协议”一起理解。同一个 `MaxOut=20` 在 RS 力矩命令、角度外环速度目标或自定义代码量中含义不同。

**改进建议：** 调参前保存固件 commit、设备配置、负载、供电和测试波形。仅改变一个参数族，并保留可恢复基线。本阶段不改任何参数。

## 2. 44 项关键运行参数

PID 的 31 个活动实例及全部 `Kp/Ki/Kd/IntegralLimit/MaxOut` 见 [08-control-algorithms.md](08-control-algorithms.md)。下表另外列出 44 项对运行、限位、通信和调试最关键的参数。

| # | 参数 | 当前值 | 单位 | 定义位置 | 使用位置 | 调大影响 | 调小影响 | 风险 |
| ---: | --- | ---: | --- | --- | --- | --- | --- | --- |
| 1 | SYSCLK | 168 | MHz | 两工程 `Src/main.c:155-204` | CPU/DWT/外设时钟 | 算力/时序改变 | 算力下降 | 必须重算外设时钟，勿单改 |
| 2 | APB1 时钟 | 42 | MHz | 同上 | CAN/UART/TIM 时钟树 | 外设时序改变 | 同左 | CubeMX 系统参数 |
| 3 | RTOS tick | 1000 | Hz | `Inc/FreeRTOSConfig.h` | `osDelay()` | 延时分辨率提高 | 分辨率降低 | tick 开销/周期语义 |
| 4 | FreeRTOS heap | 15360 | byte | `FreeRTOSConfig.h` | 动态任务/对象创建 | 可容纳更多对象 | 可能创建失败 | 未发现 malloc-failed hook |
| 5 | MotorTask delay | 1 | tick | `application/robot_task.h:37-45` | 电机计算/发送 | 周期变慢 | CPU/CAN 负载增大 | 相对延时会漂移 |
| 6 | RobotTask delay | 5 | tick | `robot_task.h:50-57` | 命令、机械臂/底盘业务 | 控制响应变慢 | 负载增大 | 目标与反馈时序改变 |
| 7 | DaemonTask delay | 10 | tick | `robot_task.h:46-49` | 在线计数 | 超时单位变粗/变慢 | 调度负载增大 | 所有 daemon 超时随之变 |
| 8 | 遥控器 daemon | 10 | daemon 计数，约100 ms | `modules/remote/remote_control.c:148-162` | 失联清零 | 更晚停止 | 更易误判失联 | 软件 STOP 延迟 |
| 9 | 图传 daemon | 30 | daemon 计数，约300 ms | `modules/referee/rm_referee.c:248-253` | 串口重启 | 更耐短断流 | 更敏感 | 离线未清键鼠状态 |
| 10 | CANComm daemon | 50 | daemon 计数，约500 ms | 两板 `application/cmd/robot_cmd.c:121-132` | 板间在线 | 更晚发现失联 | 更易抖动 | 控制板协同停止 |
| 11 | DJI daemon | 2 | daemon 计数，约20 ms | `DJImotor/dji_motor.c:218-223` | 四轮在线 | 旧反馈使用更久 | 误判风险 | 上层聚合逻辑需审查 |
| 12 | DM daemon | arm 5 / chassis 10 | daemon 计数，约50/100 ms | `DMmotor/dmmotor.c:195` | 关节在线 | 同上 | 同上 | 两板策略不同；错误码未统一上送 |
| 13 | RS daemon | 30 | daemon 计数，约300 ms | `RSmotor/rsmotor.c` | RS 在线 | 停止更迟 | 更敏感 | 与其他电机超时不一致 |
| 14 | LK daemon | 5 | daemon 计数，约50 ms | `LKmotor/` | LK 在线 | 旧反馈使用更久 | 更敏感 | 控制函数未统一停机 |
| 15 | WBUS 原始范围/中位 | 364..1684 / 1024 | count | `remote_control.h:37-47` | `wbus_to_rc()` | 范围不是调参项 | 同左 | 必须与遥控器实测相符 |
| 16 | 通道有效阈值 | 671 | count(abs) | `remote_control.c` 解析 | 异常值置零 | 接受更大异常 | 可能截掉有效端点 | 无显式小死区 |
| 17 | joint_0 限位 | -3.10..3.10 | rad | arm `robot_cmd.c:858-867` | `MotorAngleLimit()` | 工作区扩大 | 工作区缩小 | 机械碰撞 |
| 18 | joint_1 限位 | -1.47..1.39 | rad | 同上 | 同上 | 同上 | 同上 | 零点/方向须实机确认 |
| 19 | joint_2 限位 | -2.43..2.94 | rad | 同上 | 同上 | 同上 | 同上 | 机械碰撞 |
| 20 | joint_3 限位 | -3.14..3.14 | rad | 同上 | 同上 | 同上 | 同上 | 线缆扭转风险 |
| 21 | joint_4 限位 | -1.57..1.57 | rad | 同上 | 同上 | 同上 | 同上 | 机械碰撞 |
| 22 | joint_5 限位 | -3.14..3.14 | rad | 同上 | 同上 | 同上 | 同上 | 累计角/零点依赖 |
| 23 | joint_6 集中限位 | 无 | — | `MotorAngleLimit()` 未处理 `[6]` | 夹持/末端速度目标 | — | — | P1：需上层与硬件约束 |
| 24 | 图传 yaw duty | 0.05..0.13 | duty | arm `robot_cmd.c:875-880` | PWM 舵机 | 行程扩大 | 行程缩小 | 端点堵转 |
| 25 | 图传 pitch duty | 0.055..0.09 | duty | 同上 | PWM 舵机 | 同上 | 同上 | 同上 |
| 26 | 底盘关节目标0 | 0.05..5.8 | rad | arm `robot_cmd.c` 板间目标限制 | 底盘关节0 | 工作区扩大 | 缩小 | 机械含义未确认 |
| 27 | 底盘关节目标1 | -5.8..-0.05 | rad | 同上 | 底盘关节1 | 同上 | 同上 | 同上 |
| 28 | 底盘关节目标2 | 0.12..0.9 | rad | 同上 | 底盘关节2 | 同上 | 同上 | 同上 |
| 29 | 底盘关节目标3 | -0.9..-0.12 | rad | 同上 | 底盘关节3 | 同上 | 同上 | 同上 |
| 30 | 履带事件速度 | ±4000 | 代码速度量 | 两工程 `robot_cmd.c` 事件处理 | 两履带目标 | 更快/更大命令 | 更慢 | 物理单位未确认 |
| 31 | WHEEL_BASE | 400 | 源码未声明 | chassis `robot_def.h:43` | 旋转几何宏 | 旋转分量增大 | 减小 | 需 CAD/单位确认 |
| 32 | TRACK_WIDTH | 377 | 源码未声明 | `robot_def.h:44` | 同上 | 同上 | 同上 | 需 CAD/单位确认 |
| 33 | RADIUS_WHEEL | 60 | 源码未声明 | `robot_def.h:47` | 周长宏，当前轮速式未用 | 换算变化 | 同左 | 容易误以为已参与控制 |
| 34 | 功率设定 | 105 | 模型代码量 | `power_control.c:35` | 目标缩放 | 更少限功 | 更早限功 | 非实时裁判功率；重复缩放疑点 |
| 35 | 轨迹速度分段阈值 | 0.8 | rad | `trajectory_planning.c:13-34` | `compute_segment_time()` | 更多段用低速 | 更多段用高速 | 段时长突变 |
| 36 | 轨迹低速 | 0.5 | rad/s | 同上 | 小角差段 | 更快 | 更慢 | 未限加速度 |
| 37 | 轨迹高速 | 1.8 | rad/s | 同上 | 大角差段 | 更快 | 更慢 | 机构/控制带宽 |
| 38 | 轨迹时间余量 | 0.1 | s | 同上 | 每段 T | 更慢更平滑 | 更快 | T 过小致大加速度 |
| 39 | 自定义控制器滤波新值权重 | 0.05 | 比例 | arm `robot_cmd.c` | 目标一阶平滑 | 响应快、滤波弱 | 响应慢、滤波强 | 延迟与操控感 |
| 40 | INS `AccelLPF` | 0.0085 | s（公式时间常数） | chassis `ins_task.c:113` | 运动加速度低通 | 更平滑/更滞后 | 更敏捷/更噪 | `dt` 与单位须一致 |
| 41 | BMI088 `RefTemp` | 40 | °C | chassis `ins_task.c:35` 附近 | 温度 PID | 器件更热 | 温稳可能变差 | 需传感器额定资料 |
| 42 | FOLLOW PID MaxOut | 9000 | `wz` 代码量 | chassis `chassis.c:255-265` | 相对角跟随 | 旋转命令更大 | 跟随变慢 | 负 Kp/Kd 与方向耦合 |
| 43 | 四轮速度 PID MaxOut | 15000 | DJI 协议值 | chassis `chassis.c:96-137` | 四轮电流命令 | 电流/冲击增大 | 可用推力下降 | 不等于 A 或 N·m |
| 44 | 履带速度 PID MaxOut | 16000 | DJI 协议值 | chassis `chassis.c:114-135` | 两台履带 M3508 电流命令 | 电流/冲击增大 | 可用推力下降 | 协议量上限需手册 |

## 3. PID 调试要点

**代码事实：** PID 使用 DWT 实测 `dt`，绝大多数电机环开启梯形积分、积分限制和对测量微分。停止时不同驱动清积分行为不一致，模式切换没有统一重置接口（见 [08-control-algorithms.md](08-control-algorithms.md)）。【已确认｜A级】

**原理解释：**

- `Kp` 增大通常提高响应和刚度，也提高振荡、噪声和饱和风险。
- `Ki` 消除稳态误差，但会积累并导致过冲；当前有条件积分，但复位不一致。
- `Kd` 抑制快速变化，也会放大测量噪声；当前按测量微分。
- `MaxOut` 是安全边界，不是“性能旋钮”。在闭环不稳定时增大它通常让后果更严重。

**改进建议：** 先确认反馈方向与单位，再从小输出限幅开始。速度环稳定后才允许位置外环；任何符号错误都应先修正映射，不能靠负增益偶然抵消。

## 4. 安全调试顺序

以下顺序是风险递增的检查框架，不替代现场安全规程：

1. 断开动力电源，机械臂被可靠支撑；仅给逻辑板安全供电。
2. 只观察传感器原始值、CAN/UART 帧和 daemon 在线状态。
3. 人工缓慢移动机构，核对编码器回绕、零点、单位和反馈方向。
4. 在断能条件下核对软件电机方向标志与期望符号。
5. 由有经验人员建立硬件急停、机械限位、限流和最低输出上限。
6. 抬轮/卸载并固定机构，单电机短时使能测试。
7. 先速度环，再位置环；记录目标、反馈、输出、积分和 `dt`。
8. 单关节在安全工作区测试，确认 STOP 与失联能覆盖输出。
9. 多关节低速联动，检查供电、线缆和碰撞空间。
10. 离线验证运动学和轨迹，再在有经验人员监管下限速实机验证。
11. 最后验证整机模式切换、两板失联和异常恢复。

步骤 5—11 必须由熟悉实机的人员执行。最终审查确认自动动作状态表越界、底盘预初始化使能和爬台退出不可达等 P0；完成修复、独立评审和无动力验证前不得用于实机教学实验。详见 [11-safety-analysis.md](11-safety-analysis.md)。

## 5. 工具链与工程导入

| 项目 | 机械臂 / 底盘现状 | 证据 |
| --- | --- | --- |
| MCU | STM32F407IGH6 | 两个 `Engineer.ioc`、链接脚本【已确认｜A级】 |
| CubeMX 线索 | 6.12.1，`KeepUserCode=true`，目标 Makefile | `Engineer.ioc:242` 附近【已确认｜A级】 |
| 构建系统 | GNU Make | 两工程 `Makefile`【已确认｜A级】 |
| 编译器 | GNU Arm Embedded `arm-none-eabi-gcc` | `Makefile:181-182`【已确认｜A级】 |
| 链接脚本 | `STM32F407IGHx_FLASH.ld` | `Makefile`【已确认｜A级】 |
| 输出 | `build/ENGINEER.elf/.hex/.bin` 和 `.map` | `Makefile:33,318-361` 附近【已确认｜A级】 |
| VS Code | Cortex-Debug、make task、RTT client 配置 | `.vscode/tasks.json`、`.vscode/launch.json`【已确认｜A级】 |
| 调试器 | CMSIS-DAP/OpenOCD、J-Link；Ozone 配置存在 | `openocd_dap.cfg`、`launch.json`、`.jdebug`【已确认｜A级】 |
| STM32CubeIDE | 无 `.project/.cproject`，只能按现有 Makefile 工程导入/配置 | 全仓工程文件搜索【已确认｜A级】 |
| Keil | 未发现 `.uvprojx/.uvoptx` | 全仓工程文件搜索【代码中未确认｜D级】 |
| ST-Link | 未发现专用 OpenOCD/launch 配置 | 配置文件搜索【代码中未确认｜D级】 |

**工程导入：** 最可靠入口是带 GNU Arm 工具链的命令行或 VS Code 工作区。若使用 CubeIDE，应选择 Existing Code/Makefile 类型并让它调用仓库 Makefile；不要重新用 CubeMX 生成整个工程覆盖自研文件。Keil 不能直接打开当前仓库，没有证据支持一键导入。

## 6. 编译命令与本次验证

分别进入工程目录执行：

```powershell
cd Firmware/Engineer_arm
make -j16

cd ../Engineer_chassis
make -j16
```

只读预演可用：

```powershell
make -n all
```

最终审查于 2026-07-23 重新执行同一组只读检查，结果如下：

| 命令 | 结果 | 结论 |
| --- | --- | --- |
| 两工程 `make -n all` | 成功展开编译、链接、objcopy 命令 | Makefile 依赖图可解析【已确认｜A级】 |
| `arm-none-eabi-gcc --version` | 当前环境报 `Binary not found on PATH` | 缺少/未配置 GNU Arm 工具链，属环境问题 |
| 实际 `make` | 未执行 | 已知编译器缺失，且不为“通过构建”修改源码 |

Makefile 当前包含 `-Og -g -gdwarf-2 -Werror`，并定义 `DISABLE_LOG_SYSTEM`。因此源码中的日志调用不等于当前固件会输出 RTT 日志。

## 7. `.elf`、`.hex`、`.bin`

| 文件 | 作用 | 适合场景 |
| --- | --- | --- |
| `.elf` | 含分段、符号和调试信息 | GDB/Cortex-Debug/Ozone、定位函数和变量 |
| `.hex` | 带地址的 Intel HEX 文本记录 | J-Flash/常规烧录工具，地址由文件携带 |
| `.bin` | 原始连续字节，不带地址 | OpenOCD/生产工具；必须另外指定基址，Makefile 使用 `0x08000000` |

两工程输出文件都叫 `ENGINEER.*`，必须保留所在工程目录，避免把机械臂镜像烧到底盘板或反之。

## 8. 烧录与 GDB 调试

仓库 Makefile提供：

```powershell
make download_dap
make download_jlink
```

其中 CMSIS-DAP 路径由 OpenOCD 配置把 `.bin` 写入 `0x08000000`；J-Link 路径使用仓库 J-Flash 配置和 `.hex`。这些命令本阶段没有执行。

安全烧录流程：

1. 断开动力，确认目标板、MCU `STM32F407IG`、SWD 电压与地线。
2. 明确当前目录是 `Engineer_arm` 还是 `Engineer_chassis`，核对镜像时间和哈希。
3. 先连接调试器并读取芯片 ID，不使能任何电机。
4. 使用 VS Code `launch.json` 中的 CMSIS-DAP/OpenOCD 或 J-Link Cortex-Debug 配置加载对应 `build/ENGINEER.elf`；或由有经验人员执行 Makefile 下载目标。
5. 复位后停在 `main()`，先观察初始化和 STOP 状态，再决定是否继续运行。

`stm32.jflash` 的内容需要在真实 J-Flash 版本中再核对，不能仅凭文件存在认定可直接量产使用。Ozone 配置选择 STM32F407IG、SWD 4 MHz，可用 ELF 做源码级调试。仓库没有已确认的 ST-Link 路径。

## 9. 日志、串口和 CAN 调试

**串口/RTT：** BSP 日志基于 SEGGER RTT 线索，但 Makefile 定义 `DISABLE_LOG_SYSTEM`；若构建产物没有日志，先核对预处理结果，而不是先怀疑任务没运行。USART1/3 已用于图传/遥控器，不能随意拿作文本日志，否则会破坏协议。

**CAN：** 两总线配置为 1 Mbit/s。建议断能后用隔离 CAN 分析仪检查终端电阻、错误帧、ID、DLC、周期和端序，再检查控制值；协议表见 [07-communication-protocols.md](07-communication-protocols.md)。不要向总线重放 enable/torque 帧。

**GDB/Cortex-Debug：** 用 ELF 符号在 `main()`、`RobotInit()`、`RobotCMDTask()`、CAN/UART 回调和 `PIDCalculate()` 设置断点。中断断点会改变实时性；检查控制周期宜优先用 DWT 变量/无侵入采样，避免长时间停在电机已使能状态。

## 10. 调参记录模板

每次由有经验人员执行的实机调试至少记录：固件 commit、控制板/电机型号、机械负载、供电限流、模式、目标/反馈单位、周期 `dt`、参数旧值/新值、输出峰值、停止方式和异常。没有这组上下文的“某个 PID 好用”不可迁移到另一关节。
