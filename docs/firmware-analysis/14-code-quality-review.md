# 代码质量、证据一致性与最终验收审查

> 审查日期：2026-07-23；基线提交 `d7aa4214765a36a291678e58afd736484ac28083`。  
> 仅审查和修正文档；没有修改任何 `Firmware/` 源码、参数、`.ioc` 或构建配置。  
> P0–P3 是技术风险等级；许可证结论同时见 [15-license-audit.md](15-license-audit.md)，不构成法律意见。

## 1. 问题统计

| 等级 | 数量 | 含义 |
| --- | ---: | --- |
| P0 | 5 | 可能导致人身伤害、机械碰撞或严重设备损坏；实机前必须处理 |
| P1 | 7 | 可能导致错误输出、失控、崩溃或严重功能异常 |
| P2 | 11 | 影响实时性、可靠性、可维护性或合规治理 |
| P3 | 6 | 一般结构、可读性、可移植性或文档治理改进 |
| 合计 | 29 | 静态审查结果，不等于缺陷已在实机复现 |

## 2. P0 问题

| ID | 问题 | 影响 | 触发条件 | 文件和函数 | 代码依据 | 当前保护 | 建议 | 验证方法 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| P0-01 | 两个自动状态机把 `fsm.size` 固定为 47，但表分别只有 24/7 项 | 越界读取状态表；匹配到伪造项时可能调用越界读出的函数指针或写错误目标 | 自动事件进入 `FsmEventHandle()` | 两工程 `modules/fsm/auto_action.c:51-72,137`；`modules/fsm/fsm.c:FsmEventHandle()` | `FsmEventHandle` 以 `i < pFsm->size` 访问 `pActTable[i]`；实际数组长度不等于 47 | 【代码中未发现边界保护】 | 由数组长度生成 size；事件/状态做范围校验；状态机主机单测 | ASan/UBSan 枚举所有 state×event；硬件前验证无越界 |
| P0-02 | 板间 CAN 超时只重置分包状态，底盘继续使用旧命令 | 机械臂板断电/断线后，底盘可能保持非零轮速、履带或姿态目标 | 最后一个有效命令非零后链路中断 | 两工程 `modules/can_comm/can_comm.c:73-78,126-135`；底盘 `application/cmd/robot_cmd.c:562-635` | `CANCommLostCallback()` 不清解包数据；活动任务不调用 `CANCommIsOnline()` | CRC8 只拒绝坏帧，不处理陈旧帧 | 接收端超时原子切 STOP/零目标；发送端也监测反馈超时 | CAN 断线、单向断线、总线关闭故障注入并抓包/测轮端输出 |
| P0-03 | 底盘 `motor_init_finish==0` 分支仍使能四个行走电机并采用 `vx/vy/wz` | 初始化未完成或反馈缺失时仍可能移动 | 上电/初始化阶段收到或保留非零速度 | `Engineer_chassis/application/chassis/chassis.c:355-457`，`ChassisTask()` | `else` 分支 `DJIMotorEnable()` 四次并复制速度，随后仍执行 `MecanumCalculate()` | 姿态/履带未在该分支使能，但四轮未受保护 | 初始化前全部执行器保持硬失能与零报文；使用单向安全状态机 | 上电、拔掉任一反馈、随机 RAM 初值条件下抓取 0x200/0x1FF 输出 |
| P0-04 | 两板 `MotorCmdDataInit()` 的局部 `isALLone` 未初始化；底盘另在约 50 s 后强制完成 | 初始化完成标志不确定或在反馈缺失时被强制置位，可能提前使能 | 任意启动；底盘计数超过 10000 次 | 两工程 `application/cmd/robot_cmd.c:815-855` / `:516-540` | `uint8_t isALLone;` 后直接 `+=`；底盘条件含 `init_time++ > 10000` | 反馈非零时置各电机 flag，但无法消除未定义初值 | 每周期显式置零；不得以时间替代反馈/零点有效性；逐电机故障位 | 编译器告警 `-Wuninitialized`；RAM 填充模式、缺反馈和重启矩阵测试 |
| P0-05 | 爬台退出计数使用 `uint8_t`，却判断 `delay_time < 300` | 计数 255 后回绕，退出/清 `start_track` 分支不可达；持续写履带 6000、`vy=9000` | 进入 `START_CLIMB` 且轨迹完成 | `Engineer_chassis/modules/fsm/auto_action.c:200-249`；`application/cmd/robot_cmd.c:461-508` | `static uint8_t delay_time` 与 300 比较；`start_track=0` 位于不可达路径之后 | `REFORMAT` 可改变事件，但不能证明始终可用 | 使用绝对时间/足够宽计数，加入独立最大动作时间和人工撤销 | 主机单测运行 >300 周期；断通信、撤销、传感器异常组合测试 |

## 3. P1 问题

| ID | 问题 | 影响 | 触发条件 | 文件和函数 | 代码依据 | 当前保护 | 建议 | 验证方法 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| P1-01 | 浮点 PID 误差/输出使用整数 `abs()` 而非 `fabsf()` | 小于 1 的误差会先转换为整数 0；死区、积分策略和饱和判断偏离设计 | 浮点误差或临时输出非整数，影响全部 31 个活动 PID | 两工程 `modules/algorithm/controller.c:29-43,149-209` | 5 处 `abs(float)`；遥控器的整数 `abs` 不在此问题内 | 最终输出仍有数值钳位 | 使用 `fabsf`；为死区、积分限幅、正负饱和建立单元测试 | 主机输入 0.01/0.5/0.99/1.01、NaN、±Inf，对照公式 |
| P1-02 | 机械臂存活判定仅覆盖 4/7 台且条件为“四台同时离线” | 单电机或部分电机丢反馈时整机仍标 ALIVE，旧目标/状态继续参与上层逻辑 | 1–3 台受监测电机离线，或未列入的电机离线 | `Engineer_arm/application/gimbal/gimbal.c:317-350`，`GimbalAliveCheck()` | 离线条件用四个 `&&`；注释也写“仅检测四个电机” | 各驱动有 daemon，但上层整机保护不一致 | 逐电机 online 位；任一关键轴离线进入定义明确的降级/STOP | 逐个拔反馈、组合拔反馈并观察模式/所有 CAN 输出 |
| P1-03 | 图传 UART 解析不使用实际 `Size`，`0xA9 0x53` 路径无长度/CRC校验 | 短帧、粘包或噪声可能制造错误按键沿和控制动作 | 任意以两字节头开头但不足 21 B 的 DMA idle 帧 | `bsp/usart/bsp_usart.c:98-120`；`modules/referee/rm_referee.c:182-208` | 回调签名不带 Size；固定从偏移 10 复制 11 B；daemon 在解析前重载 | 只检查两个头字节 | 长度/CRC/序号验证后再发布；失败不喂 daemon，不沿用按下状态 | UART 模糊测试、截断/拼接/翻转 bit；检查控制结构保持安全 |
| P1-04 | 消息中心队首更新表达式具有未定义行为 | 编译器/优化变化可破坏索引；QUEUE_SIZE 扩大后可能错读/越界 | 每次成功 `SubGetMessage()` | 两工程 `modules/message_center/message_center.c:84-116` | `sub->front_idx = (sub->front_idx++) % QUEUE_SIZE` 同一标量无序读写 | 当前 `QUEUE_SIZE=1` 偶然掩盖大部分表现 | 改为 `(front_idx + 1) % QUEUE_SIZE`；增加参数/并发保护 | UBSan + 不同优化级别 + QUEUE_SIZE 1/2/8 单测 |
| P1-05 | 初始化期大量 `malloc()` 后未判空即 `memset`/解引用 | Heap 不足直接 HardFault，执行器安全态不受保证 | 堆不足、碎片或实例数量配置错误 | `bsp_can.c:102-103`、`bsp_usart.c:49-60`、`can_comm.c:82-102`、各电机/消息中心注册函数 | 多处相同模式；无 `vApplicationMallocFailedHook` | 注册数超限有的会死循环，但分配失败无保护 | 关键对象静态分配或集中 checked allocator；失败进入安全故障 | 缩小 heap、逐次注入 malloc failure，验证无输出且保留诊断 |
| P1-06 | Fault/断言只关中断或死循环，无执行器撤销、故障快照、看门狗恢复 | 软件异常后电机驱动器可能继续执行最后命令，且无法定位原因 | HardFault、BusFault、UsageFault、`configASSERT` | 两工程 `Src/stm32f4xx_it.c` Fault handlers；`Inc/FreeRTOSConfig.h:122` | 处理器循环；IWDG/WWDG 未启用 | 电机驱动器自身保护不能由本仓库证明 | 独立硬件 enable；Fault 首先撤销；保存 SCB/栈并受控复位 | 人工触发各 Fault/断言，示波器验证失能时延和复位记录 |
| P1-07 | PID 对 `dt==0`、NaN/Inf 和非法参数无防御 | 微分除零、NaN 穿过比较/钳位，最终形成异常协议值 | DWT 时间异常、反馈/目标非有限、参数损坏 | 两工程 `modules/algorithm/controller.c:155-194` | `Kd*(Err-Last_Err)/dt`；无 `isfinite`/dt 下限；比较对 NaN 失效 | 协议打包有范围宏，但 NaN 转整数行为不能作为保护 | 入口验证、dt 上下限、非有限即零输出并置故障 | 参数/时间模糊测试；开启浮点异常/UBSan，抓取输出帧 |

## 4. P2 问题

| ID | 问题 | 影响 | 触发条件 | 文件和函数 | 代码依据 | 当前保护 | 建议 | 验证方法 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| P2-01 | 所有周期任务使用相对 `osDelay()` | 执行时间累积为周期漂移，名义 1/5/10 ms 不稳定 | 长计算、CAN 忙等、RTT/UI 抢占 | 两工程 `application/robot_task.h` 及电机任务 | 未找到 `vTaskDelayUntil` | PID 用 DWT 实测 dt，部分缓解但不保证 deadline | 绝对周期调度并记录 deadline miss | GPIO/Trace 测平均、P99、最大周期 |
| P2-02 | CANComm、遥控器、图传、电机测量在 ISR/任务间无一致快照 | 多字段读到不同帧，事件和数值组合不一致 | ISR 在任务复制/读取过程中抢占 | `can_comm.c`、`remote_control.c`、`rm_referee.c`、各 motor decode | 结构体 `memcpy`/逐字段写，无临界区、双缓冲或序号 | 单核 MCU 降低但不消除多字节撕裂 | 双缓冲 + 发布序号；短临界区复制 | 高速注入带递增序号，检测混合帧 |
| P2-03 | CAN ISR 命中一帧后退出，未在同次回调持续清空 FIFO | 突发反馈延迟并增加溢出风险 | FIFO 中同时有多帧且第一帧匹配实例 | 两工程 `bsp/can/bsp_can.c:173-203` | `while` 内匹配分支 `return` | 硬件可能再次触发 pending，但延迟/溢出未测 | 每次有界清空 FIFO，统计 overrun 和 ISR 预算 | 满载 CAN 注入，测 FIFO fill/overrun/反馈年龄 |
| P2-04 | CAN 发送忙等且多数调用者忽略返回值 | 任务抖动、静默丢命令 | 邮箱满或 bus-off | `bsp_can.c:129-151`；`can_comm.c:116-123`；各 motor send | 1 ms 软件超时与警告日志 | 日志宏可能禁用，未形成上层故障 | 非阻塞队列/发送完成通知；累计失败触发安全态 | bus-off、100% 总线负载、邮箱占满测试 |
| P2-05 | PID 积分限幅辅助函数使用跨实例 `static` 临时量 | 两任务抢占时串用中间量，积分处理偶发错误 | RobotTask 与 MotorControlTask 同时执行 PID | 两工程 `controller.c:38-49` | `static float temp_Output, temp_Iout` | 无锁；当前任务同优先级仍可时间片/中断切换 | 改自动局部量或实例字段 | 强制抢占插桩和 ThreadSanitizer 风格主机测试 |
| P2-06 | 控制、INS、daemon、UI 等活动任务均为 Normal 优先级 | 非关键任务可能推迟控制/守护处理 | UI/INS/CAN 负载升高 | 两工程任务创建处 | 12 个活动任务实例均 Normal | 时间片调度提供公平性，不提供控制优先级保证 | 基于测量做速率单调/响应时间设计 | Trace 查看 ready/running、worst-case latency |
| P2-07 | 未启用栈溢出、malloc failure、运行时间和 CPU 负载监测 | 资源耗尽无法提前发现，故障难复现 | 深调用、日志、库升级、任务增多 | `FreeRTOSConfig.h` 和应用层 | 未定义两个 hook；内核默认值为 0；无 runtime stats 使用 | `configASSERT` 仅停机 | 启用 hook/watermark/runtime stats，持久化复位原因 | 压测栈/堆并核对告警阈值 |
| P2-08 | 底盘功率缩放在 4 个驱动电机子步中重复作用于整组目标 | 同一 RobotTask 周期内缩放 4 次，限制强度依调度次数而非控制模型 | 功率控制启用且系数 <1 | `Engineer_chassis/modules/motor/DJImotor/dji_motor.c:DJIMotorControl()` 与 `application/chassis/chassis.c` | 组电流数组在每个电机迭代中传入 `calfromPID()` | 最终 PID/协议限幅仍存在 | 每个整组周期只计算一次功率分配，再发送 4 路 | 离线回放同一输入，对比一次/四次缩放与功率 |
| P2-09 | 板间协议直传 float/enum，无版本、序号、时间戳 | 编译选项/结构修改导致静默不兼容；无法判断数据年龄 | 两端版本不同、short-enum、ABI/端序变化 | 两工程 `application/robot_def.h`；`robot_cmd.c:121-132` | 发送 41/接收21 B 和反向一致；CRC8 检错 | 固定长度能发现部分结构变化 | 显式字节协议、版本/序号/时间戳、静态断言 | 两工具链交叉编码；版本不匹配拒收测试 |
| P2-10 | 活动自研/继承代码中存在 HNU YueLu EC “all rights reserved” | 仓库级 Apache 声明不能自动覆盖这些文件，分发/修改授权不确定 | 对外发布源码/固件或接受外部贡献 | `application/robot_def.h`、`modules/algorithm/controller.c`、`modules/motor/DJImotor/dji_motor.h` 等 | 根 `LICENSE`/`Firmware/LICENSE` 明确排除既有声明 | `NOTICE` 已提醒混合来源 | 获取权利人书面授权/来源清单；逐文件 SPDX | 由权利人/法务核查来源、贡献历史和许可链 |
| P2-11 | ST USB 库使用 SLA0044，含仅限 ST 设备及 Open Source Terms 限制 | 不能把整个固件简单描述成单一 Apache-2.0；移植非 ST 平台有合规风险 | 重分发 USB 源/二进制或移植平台 | 两工程 `Middlewares/ST/STM32_USB_Device_Library/LICENSE.txt` | 条款 4 限 ST 设备，条款 5 限制引入 Open Source Terms | 根 LICENSE 已排除第三方条款 | 发布清单单列 SLA0044；法务确认组合/分发方式 | 发布前许可证清单和目标芯片审查 |

## 5. P3 问题

| ID | 问题 | 影响 | 触发条件 | 文件和函数 | 代码依据 | 当前保护 | 建议 | 验证方法 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| P3-01 | 每工程同时保留顶层 `Src/Inc` 和 `Core/Src/Core/Inc` 重复树 | 开发者容易修改未参与当前 Makefile 的副本 | 修复写到 `Core/` 后构建顶层 `Src/` | 两工程 Makefile 源清单与重复目录 | 当前 Makefile 不编译 Core 树 | 文档已指出真实入口 | 删除前先溯源或生成校验；CI 检查重复差异 | 修改哨兵注释后 `make -n`/依赖清单确认（不在本审查执行） |
| P3-02 | 两工程编译大量未由 `RobotInit()` 初始化的业务/驱动代码 | 增大镜像、告警面和误读概率 | 模块被复制后未裁剪 | Makefile 与两工程 `application/robot.c` | “被编译”不等于“活动” | 文档已区分活动/非活动 | 用目标清单显式选择模块；启用 section GC 报告 | map 文件与调用图审计 |
| P3-03 | 控制参数和单位分散为魔法数字 | 调参难审查、单位/机械版本易错配 | 机械版本或电机更换 | `robot_cmd.c`、`gimbal.c`、`chassis.c`、motor headers | 限位、步长、6000/9000、PID 分散 | `10-parameters-and-tuning.md` 已建立字典 | 集中只读参数表，字段带单位/版本/来源 | 生成参数清单并与机械 BOM/标定签名比对 |
| P3-04 | `gimbal` 名称承载机械臂，两个工程大量复制同名模块 | 职责混淆、修复易只改一份 | 维护者按名字或单工程修改 | `application/gimbal`、双份 `modules/`/`bsp/` | 文档已有术语说明 | 无代码级共享来源 | 提炼共享库，业务名称逐步去歧义，建立同步测试 | 两树语义 diff 和双工程 CI |
| P3-05 | 扩展 CAN ID 使用 C 位域和指针重解释 | 依赖编译器位域布局、别名和端序，可移植性差 | 编译器/ABI/优化改变 | `RSmotor/rsmotor.h:53-82`、`rsmotor.c:RSMotorDecode/RSMotorWriteParam` | 结构体强转/按字节复制解析 29-bit ID | 当前同一 GCC/MCU 组合可能稳定 | 用移位/掩码显式打包，加入 `_Static_assert` 和字节向量测试 | GCC/Clang 多优化级别编码 golden vectors |
| P3-06 | 若干命名作者算法/厂商驱动只有作者名或无明确许可 | 归属和可修改/再分发范围不清 | 对外发布或二次许可 | `QuaternionEKF.*`、`kalman_filter.*`、部分 DM/RS/HT/YS/编码器驱动 | 文件头有作者/版本但未见 SPDX 或随附专属许可 | `Firmware/LICENSE` 仅覆盖有权授权的原创贡献 | 建立 SBOM/来源 URL/授权证据；无法确认者隔离 | 权利人确认、历史提交和原始发行包比对 |

## 6. 证据复核结果

| 抽查范围 | 覆盖 | 结果 | 主要依据/修正 |
| --- | ---: | --- | --- |
| 操作映射 | 14 条（至少 10 条） | 变量和活动条件可追溯；物理面板名称仍为 D 级 | `RemoteControlSet()`、`MouseKeySet()`、`AutoActionEventTransfer()`；操作手文档已重分组 |
| 控制模式 | 全部活动 `robot_state`、gimbal/chassis mode、自动事件 | 枚举与分支一致；自动 FSM 风险升级为 P0 | 两工程 `robot_def.h`、`robot_cmd.c`、`auto_action.c` |
| 机械臂主链 | 7 电机 + 2 舵机 | 输入→目标→限位→控制器→CAN→反馈闭合 | `05-arm-control.md`；`robot_cmd.c`、`gimbal.c`、各 motor driver |
| 底盘主链 | 4 轮 + 2 履带 + 5 姿态电机 | 链路闭合；补充“未初始化仍使能四轮”P0 | `06-chassis-control.md`；`chassis.c:355-475` |
| 板间协议 | 2 个逻辑方向 | ID、总线、字段长度一致；两端 `daemon_count=50`，按名义 10 ms 守护任务推导约 500 ms | 两工程 `robot_cmd.c:121-132`、`can_comm.c`、daemon 10 ms |
| 主要电机 CAN ID | 全部活动实例 | 与初始化配置和发送驱动一致 | 两工程 `gimbal.c`/`chassis.c`、四类 motor driver |
| PID 实例 | 31 个活动环 | 数量保持 13+18；新增 `abs(float)` 实现缺陷 | 两工程电机初始化配置与 `controller.c` |
| 任务周期 | 每板 6 个，共 12 个活动任务实例 | 延时值可确认；严格周期不可确认 | `robot_task.h`、motor task init、`FreeRTOSConfig.h` |
| 安全保护 | 本文矩阵全部条目 | 事实/缺失已区分；不再把软件 STOP 写成硬急停 | `EmergencyHandler()`、驱动 stop、daemon、Fault handlers |
| 参数单位 | 44 项关键参数复核 | 代码有明确单位才保留；机械尺寸/物理方向继续 D 级 | `10-parameters-and-tuning.md`；协议/driver 注释与换算 |

### 本阶段修正的重大文档错误

1. 底盘 DM daemon 从“约 50 ms”修正为“约 100 ms”：`reload_count=10` × DaemonTask 约 10 ms。
2. 图传 daemon 从“约 100 ms”修正为“约 300 ms”：`reload_count=30`。
3. 驱动索引路径 `modules/robotic_def/rm_referee.*`、`modules/HC05/` 修正为实际 `modules/referee/rm_referee.*`、`modules/bluetooth/HC05.*`。
4. `ImgTrans_info_t` 的定义位置修正为 `modules/referee/rm_referee.h`。
5. 软件 SA/遥控器 STOP 明确降格为“软件停止条件”，不得称为经验证的硬件急停。
6. 新增此前文档未显著标出的底盘预初始化使能、爬台计数器回绕和 `abs(float)` 三条关键缺陷。

## 7. 文档一致性验收

- 相对 Markdown 链接：最终脚本检查结果以本次回复为准；目标是 0 个失效链接。
- Mermaid：检查 fenced block 成对闭合，并对每块首行类型做静态检查；未安装 Mermaid CLI 时不声称完成渲染器级验证。
- 标题/空章节：检查重复同级标题和空章节；同名“代码事实/原理解释/改进建议”属于各节教学模板，不视为冲突。
- 术语：统一使用“机械臂板/底盘板”“软件 STOP”“板间 CANComm”“活动/已编译但未初始化”。
- 证据：A 级只用于代码直接可闭合结论；物理按键、关节名称、方向、零点、机械尺寸和实机安全均保持 C/D 级。

## 8. 改进路线

| 阶段 | 建议 | 原因/优先级 | 预期收益 | 风险 | 建议验证 |
| --- | --- | --- | --- | --- | --- |
| 立即 | 处理 P0-01…P0-05，配置独立物理断能 | 直接动作/越界风险，最高 | 消除最短路径失控场景 | 修改控制逻辑可能引入回归 | 主机单测 + 无动力台架 + 独立安全评审 |
| 立即 | 板间/单电机/图传超时统一进入安全状态 | 当前保护链断裂 | 失联不再沿用旧目标 | 过严阈值可能误停 | 故障注入和总线延迟分布确定阈值 |
| 短期 | 修正 PID 数学、防 NaN/dt 异常并集中参数/单位 | 31 环共用实现 | 控制行为可预测、可测试 | 参数行为改变需重新整定 | golden-vector、仿真、单电机台架 |
| 短期 | 建立模式仲裁器、切换清积分/目标斜坡 | 当前状态分散 | 降低模式跳变 | 可能改变操作手手感 | 记录切换前后目标/反馈/输出 |
| 短期 | 启用 Fault 快照、stack/heap/CPU 监控和看门狗 | 当前停机无诊断/恢复 | 提升现场可定位性 | 喂狗设计错误会掩盖死锁 | 故障注入、复位原因持久化 |
| 中期 | 双缓冲/序号化共享数据，非阻塞 CAN 发送 | 数据撕裂和任务抖动 | 实时性与一致性提高 | 并发架构改动较大 | Trace、压力测试、协议回归 |
| 中期 | 协议版本化、显式序列化、两板兼容矩阵 | 直接传 ABI 对象 | 支持独立升级和拒绝不兼容 | 需要迁移期双协议 | golden frames、混合版本 CI |
| 中期 | 提炼共享 BSP/驱动/算法，建立双工程 CI | 两份代码易漂移 | 修复一次、双板验证 | 抽库可能改变构建布局 | 两镜像 size/map/行为对比 |
| 中期 | 自动化单元、协议、模糊、SIL 仿真 | 当前只靠静态分析 | 在硬件前捕获边界错误 | 建模成本 | CI 中固定种子和回归向量 |
| 长期 | HIL、断线/碰撞/限位/过载测试矩阵 | 软件静态证据不足 | 量化整机故障响应 | 需要安全工装 | 审批后的 HIL 与台架验收记录 |
| 长期 | 正式操作手册、安全评审和版本联动 | 当前操作内容未实机验证 | 可审计培训与维护 | 过期文档反而危险 | 固件/机械/电气/手册同版签署 |
| 长期 | 完整 SBOM、来源证明和发布合规流程 | 混合许可证/未知来源 | 降低再分发风险 | 可能需替换无法授权代码 | 法务/权利人确认，发布包扫描 |

## 9. 最终验收判断

代码学习、静态维护和后续修复定位所需的地图、控制链、算法、参数、安全与许可证入口已经齐备。【已确认｜A级】 但 5 个 P0 尚未修复，且物理急停、方向、零点、限位、遥控器映射和整机失联均未实机验证。因此操作手内容只能有条件使用，不能宣称完成正式安全验收。
