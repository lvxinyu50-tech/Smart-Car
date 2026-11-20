# 22 智能车 · test2 校赛候选程序

工程目录为 `D:\SmartCar\test2`。KEY4 驱动两轮并执行视觉循迹。首轮架空测试使用实体断电开关。软件已在 ADS 构建并用合成图像联测，真实赛道、停车距离和完整跑圈待实车签收。**十字路口默认直行，这是待现场路线规则确认的假设。**

## 打开、烧录与按键

IPS200 在待机和 KEY4 模式下自动轮换 **B MASK 道路轮廓** 与 **A GRAY 原始灰度**。默认先显示轮廓，每页约 4 s。屏幕右侧有颜色图例。[IPS200 实车读屏指南](IPS200实车调试.md)

在 ADS 中用 **File → Import → Existing Projects into Workspace** 选择本目录，选项目 `test2` 和 **Debug**。调试选择 `test2 Debug`，下载前核对目标 `Debug/test2.elf`。独立 HEX 见[烧录文件](Debug/test2.hex)，带符号 ELF 见[调试文件](Debug/test2.elf)。重新干净构建时仅对 test2 执行 Project → Clean，然后 Build Project。

| 入口 | 行为 |
|---|---|
| 上电 | IDLE，两轮为零，舵机中值 1505。可以直接在 IPS200 看真实道路和中线 |
| KEY1 稳定按下 | 连续三次 10 ms 采样后立即锁存，持续按住只交付一次停车通知。控制中断独立检查锁存，前台刷新屏幕时也能停车 |
| KEY2 稳定松开 | 独立开环，默认两轮从 300 开始，每 500 ms 加 100，最高 1200，入场起 10 s 超时归零。单位为万分之一占空比 |
| KEY3 稳定松开 | 独立直行 PI，目标 0.20 m/s。KEY2、KEY3 均可在摄像头未接时使用 |
| KEY4 稳定松开 | 进入视觉模式，先停止原控制器、归零、回中。停顿 ≥300 ms、连续 ≥200 ms 双轮零脉冲且零 PWM、连续 3 张入场后采集的可靠普通道路帧，才可运行。视觉中再按下并松开 KEY4 回到 IDLE |

**停车和人工恢复。** 按住 KEY1 可以紧急停车。普通切换先停旧控制器，再等待新模式放行。视觉运行中一帧不可靠即在控制节拍归零、回中并显示 HOLD。即使随后恢复单帧识别，仍保持停车。可用 KEY4 正常退出，重新按 KEY4 另行启动。连续两帧不可靠，或连续坏帧的采集完成时刻跨度达到 100 ms，会进入 LINE LOST。若只收到一张坏帧后就完全断帧，则按 300 ms 帧超时处理。控制任务会记住两次节拍间已发生的视觉失效。

| FAULT 显示 | 原因与操作 |
|---|---|
| KEY1 | 确认车辆已物理停稳、四键完全释放、停车通知已交付，屏显 RESET YES 后，在 ADS Expressions/Watch 对 `test_manual_reset_request` 单次写 1 |
| CAM INIT / FRAME TIME | 摄像头配置失败 / 收不到新鲜且已分析的完整帧达到 300 ms。检查供电、连接和 FR。确认停稳后可用同一人工命令回 IDLE，继续 KEY2/3 底盘测试。摄像头初始化仅在上电运行，要重新探测摄像头应修复后重启 MCU |
| LINE LOST / CROSS TIME | 检查照明、阈值、入口与出口几何，或十字穿越时限。在停稳、四键稳定释放、RESET YES 后单次写 1 |
| COAST TIME / CONFIG / REPORTED | 停稳等待超过 2 s、参数无效或已有严重底盘故障。排除原因后人工重启 MCU |

复位成功会清空控制读者积压事件、保持 IDLE。须有新的人工按键才会再次开车。急停期间的 KEY4、松开 KEY1、恢复图像均不能重启。提前写入的复位命令会被消费，满足条件后需重新写入。若视觉故障与 KEY1 同时存在，恢复还须通过按键驱动的停车交付、原始电平和消抖检查。

## 图像、调度和执行器控制权

- **上电**沿用 test1 的 clock/debug → motor、encoder、steer、key、led → test_mode → IPS200 → MT9V03X → 显示 → 双核同步 → 两路 PIT。摄像头初始化只尝试一次，`mt9v03x_init()` 返回 0 表示成功。库先尝试 SCCB，再回退 UART1。检查本次链接的 SCCB 代码可见设备标识读取有 10 次重试上限，UART 配置和状态等待使用 `MT9V03X_INIT_TIMEOUT=0x80` 的有限循环。未连接实物时的启动耗时仍需测量。
- **VSYNC/DMA5**沿用逐飞 V3.4.5 单缓冲。P02.0 下降沿进入已有 ERU3 ISR，许可通过才调用一次 SDK 的 VSYNC 回调。P02.1 的 PCLK 上升沿经 ERU2 触发 DMA5。188×120 共 22,560 个灰度字节，两段各 11,280 字节。DMA5 ISR 先调用 SDK 回调，再检查 DMA 已停止和 `mt9v03x_finish_flag`。中间分段、跨 VSYNC 的残帧和中止传输不交付。
- **帧归属**为 FREE → CAPTURING → READY → FREE。普通帧在 READY 期间只做七行二值化与识别。屏幕请求新图时，前台在 READY 所有权下复制完整灰度帧，随即释放 DMA，再用副本生成 188×120 二值图。道路搜索直接使用这份二值图的扫描行。该帧的边界、路径点和控制结果随图保存；直到本轮显示结束才请求下一份快照。整帧复制、识别及 SPI 绘制期间允许 PIT 抢占。
- **时间和模式隔离**。在最终 DMA 回调记录 `completed_ms` 和 STM0 计数，前台记录 `processed_ms`。帧龄从采集完成计算。PD 采用相邻新帧 STM0 计数差除以实际 STM0 频率，单位为秒。视觉模式进入和退出均切换代号，入场前开始的帧、计算期间退出模式的旧结果都无法得到执行器许可。
- **CPU0 前台**依次调用 `test_mode_poll()`、`vision_track_task()`，约每 200 ms 刷新 IPS。**CCU61_CH0** 10 ms 只扫描一次按键。**CCU60_CH0** 10 ms 先调用一次 `update_speed_and_distance()`，再调用一次 `test_mode_tick_10ms()`。只有该控制任务在初始化后提交舵机和电机输出，开环、直行 PI 和视觉 PI 互斥。进入有效运行时只启动一次 PI，`speed_control_set_target()` 只更新目标，原斜坡和积分连续。
- **临界区**保留控制端的急停检查、模式判定与 PWM 提交的一致性。视觉发布只复制一个固定长度结果，图像扫描、制图和 SPI 刷屏均在关中断区外。实车可在 ADS Watch 读取 `test_control_max_ticks`，除以 STM0 实际频率可得该次上电观测到的最长控制临界区工作时间。另应实测整条中断和 EN 归零时延。

## 道路、弯道与十字

灰度默认大于 120 判为白道。按图像 y 从近到远枚举每一行的连续白区间，按前一可靠行的重叠关系、预期中心、同一行上一帧位置和宽度选取。把每行实际观察到的区间与是否可用于控制分别记录。只见一侧、触边、接近全白、无区间、相近评分有歧义时均保留诊断，图像边缘不会充当缺失的道路边界。普通道路只加权可靠行，还需有近端和中远端支持。近远中线差表示像素趋势，不能换算为物理曲率或前瞻距离。

| 视觉状态 | 判据与控制 |
|---|---|
| NORMAL | 可靠行加权中线减参考列得到误差。近远中心差或偏差较大时选弯道速度 |
| CROSS_DETECT | 至少两个相邻行相对各自近期宽度向两侧扩张，近端入口仍可见。出口须有相邻两行支持，且与入口投影方向相容。证据不够时禁止输出。仅在连续新帧支持同一入口和出口时增加确认数 |
| CROSS_PASS | 连续 3 帧确认后进入。通过测得的近端入口和远端出口，在图像各采样行插值求短段中线。若出口短暂消失，仅在入口和横向扩张仍可靠时，使用新的入口位置及最近测得的道路方向有限外推 |
| REACQUIRE | 重新出现与出口相容的普通道路后，连续 3 帧确认才返回 NORMAL。回到路口形态不会重开穿越计时 |
| LOST | 发布无效结果，撤销驱动许可。穿越超时单独记 CROSS TIME。全白、全暗及单纯道路变宽均不能单独确认十字 |

十字使用图像几何和时间上限，不依赖未经标定的里程。PD 只在前台处理新帧时计算。首帧、会话改变、失效恢复、视觉状态变化和过长帧间隔会丢弃上次微分。转向符号独立可调，指令最终经 `steer_set_offset()` 和机械限位。

## 参数与屏幕

视觉参数统一位于 [vision_config.h](code/vision_config.h)。底盘时序在 [test_mode_config.h](code/test_mode/test_mode_config.h)，硬件标定在 [driver_config.h](code/driver_config.h)，原 PI 参数在 [smartcar_param.h](code/smartcar_param.h)。

| 参数 | 交付值 | 依据与现场核实 |
|---|---|---|
| 极性、阈值、参考中心 | 亮道、120/255、x=94 | 继承 test1。蓝底与白道以现场灰度对比为准 |
| y / 权重 / 宽度范围、名义宽度 | 见下表，均为图像像素 | test2 初值。支撑杆约 30° 无法确定镜头对地俯角。沿实际直道、弯道核实 |
| 关联与可信度 | 最大搜索偏移 34 px、区间容许间隔 8 px、近似评分差 8、至少 4 行 | test2 初值。可信度 Q 是可信行权重占总权重的百分比 |
| 转向 | 符号 −1、KP 2.0 占空比单位/px、KD 0、限幅 ±140 | 符号、P 和限幅沿用 test1 待实测。KD 的单位是占空比单位·s/px，使用除以秒数的微分。异常间隔 120 ms |
| 速度与弯道门限 | 直 0.15、弯 0.12、十字 0.12、上限 0.20 m/s。近远差 12 px 或绝对误差 16 px 降速 | 候选值，须实测起转和连续运行。仅转向测试设 `VISION_STEER_ONLY=1`，动力输出为零 |
| 十字 | `VISION_CROSS_STRAIGHT=1`、相对本行宽度 160%、两侧各外扩 10 px、出口投影容差 24 px | 初始规则假设。设 0 时疑似路口不放行。检测/恢复各 3 帧，外推 <100 ms，PASS 与 REACQUIRE 共用 900 ms 上限 |
| 视觉保护 | 3 张可靠起步帧、1200 ms 获取可靠道路、300 ms 帧超时、2 张连续坏帧或 100 ms 持续不可靠 | 帧超时继承 test1。其余按真实帧率、车速和停车距离复核 |
| 底盘 | 电机 17 kHz，方向修正各 1。编码器 TIM5/TIM6，11504 计数/m。舵机 100 Hz，中值 1505，机械限位 1330～1680 | 继承 test1。实际起转 PWM、转向、每米计数和制动距离待标定。原 PI、积分及输出限幅保持原值 |

| 近 → 远的 y | 110 | 100 | 90 | 80 | 70 | 60 | 50 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 权重 | 3 | 3 | 3 | 3 | 2 | 2 | 2 |
| 最小宽度 px | 12 | 12 | 10 | 9 | 8 | 7 | 6 |
| 名义宽度 px | 72 | 66 | 58 | 51 | 44 | 38 | 32 |
| 最大普通宽度 px | 114 | 106 | 94 | 84 | 74 | 65 | 56 |

IPS200 在 240×320 竖屏上居中显示 188×120 灰度或二值图，保留原始比例。B MASK 中绿色为有效左边界，红色为有效右边界，黄色为实际路径，蓝色为参考列，橙色为无效行或候选端点。采样点之间仅连接相邻有效边界。屏上 FR、阈值、边界与 CX 来自同一次分析，实速和占空比另取控制状态快照。约每 200 ms 请求一张图，每遍前台最多发送六行图像和一行变化文字。图像超时会清屏，锁存故障另列在底行。速度单位为 mm/s。[实车读屏指南](IPS200实车调试.md)

## 文件、构建与校验

本次显示升级涉及 `code/vision_config.h`、`code/vision_road.c/h`、`code/vision_track.c/h`、新增的 `code/vision_debug.h`、`code/test_mode/vision_display.c/h`、`code/test_mode/test_display.c/h` 和前台入口 `user/cpu0_main.c`。

| 范围 | 相对 test1 的变化 |
|---|---|
| 工程和调试 | 修改 `.project`、`.cproject`、`.ads/winIDEAWorkspaces/Debug/ADS.xjrf`。改名为 `test2 Debug.launch` 和 `.settings/test2-debug-includes.opt`。工作区引用、链接和调试均使用本工程，继承本地 SDK 和链接脚本 |
| 视觉 | 新增 [vision_config.h](code/vision_config.h)、[vision_road.c](code/vision_road.c)、[vision_road.h](code/vision_road.h)。修改 [vision_track.c](code/vision_track.c)、[vision_track.h](code/vision_track.h)，负责帧归属、时刻、代号与前台发布 |
| 控制和显示 | 修改 [test_mode.c](code/test_mode/test_mode.c)、[test_mode.h](code/test_mode/test_mode.h)、[test_mode_config.h](code/test_mode/test_mode_config.h)、[test_display.c](code/test_mode/test_display.c)、[smartcar_param.h](code/smartcar_param.h)。移走重复视觉参数，接入入场、故障、速度分段及显示 |
| 其他 | 更新本 README 和[实车测试清单](实车测试清单.md)，新增 build-report 阶段记录与软件检查。CPU0 入口、ISR、逐飞库、五个驱动和速度 PI 代码沿用独立副本 |

| ADS Debug 阶段 | 结果 | 可恢复快照 |
|---|---|---|
| A 独立基线，空目录构建 | 228 个 C 单元，0 错误，26 条原 UI 警告 | [日志](build-report/A.log) · [源码](build-report/A-source.zip) |
| B 摄像头与七行识别 | 0 错误，29 条警告，含 3 条像素索引窄化提示，已在 C 中修正 | [日志](build-report/B.log) · [源码](build-report/B-source.zip) |
| C 弯道和新帧 PD | 增量编译通过，0 错误、0 警告 | [日志](build-report/C.log) · [源码](build-report/C-source.zip) |
| D 十字候选，空目录构建 | 229 个不同 C 单元各编译一次、229 个汇编单元。0 错误，26 条均为旧 ui.c 的 W523。ROM 82,736 B、RAM 50,768 B | [阶段日志](build-report/D.log) · [源码](build-report/D-source.zip) |
| IPS 双页显示，空目录构建 | 230 个 C 单元，0 错误，26 条均为旧 ui.c 的 W523。ROM 83,554 B、RAM 98,366 B | [构建日志](build-report/IPS-Debug.log) · [显示回归结果](build-report/ips-display-checks.log) |

恢复阶段代码时，在独立的空工作副本保留同版 libraries，以快照中的 code、user、.settings、.ads 和根目录工程文件完整替换对应内容，并从空 Debug 重建。旧阶段不包含以后新增的源文件。

- `Debug/test2.elf` 的 SHA-256 为 `9585832962819ecc4254d76186aca4951af363f442c331beb90c74fb853bf6a7`。[ELF](Debug/test2.elf)
- `Debug/test2.hex` 的 SHA-256 为 `bfe09d2bdedc3d5682d51755e9ae20cf9339981d521a903c54406b054560e40c`。[HEX](Debug/test2.hex)
- 已核对生成的编译命令、依赖、头文件搜索、链接清单和调试配置。五个驱动各一份，`SMARTCAR_UI_KEY_READER` 仅在旧 `ui.c` 的编译命令中，旧 `car_control.c` 被排除。原 `smartcar` 的 1,453 个文件、`test1` 的 1,465 个文件操作前后 SHA-256 逐项一致。[完整前后校验清单](build-report/source-checksums.zip)
- 主机用实际驱动、速度 PI、视觉和状态机，模拟 GPIO、编码器、PWM、DMA 完成与 STM 时钟。交付配置与黑道/非零 KD/仅转向配置各通过 20 组。覆盖弯道、多区间、触边、全白全暗、十字确认及恢复、有限外推、固定总时限、重复和陈旧帧、摄像头失联、急停与旧按键重放、模式互斥。解压测试包到工程外，在解压目录运行 `python run.py D:\SmartCar\test2`，需要 Python 3 和 gcc。[可复现测试包](build-report/software-checks.zip) · [运行结果](build-report/software-checks.log)

真实画质、蓝底灰度分离、车辆姿态、低速稳定性、临界区实测耗时和完整赛道表现尚未验证。首轮操作按[架空到整圈的八步清单](实车测试清单.md)执行。
