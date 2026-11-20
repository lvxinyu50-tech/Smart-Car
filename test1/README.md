# test1 底盘与基础视觉循迹

**KEY4 现在会启动低速循迹。** 首次使用先架空车轮，在 IDLE 查看摄像头、三行边线和误差。运行中按住 KEY1 急停。在视觉模式再按下并松开 KEY4 可正常回到 IDLE。

工程根目录为 `D:\SmartCar\test1`。在 ADS 中选择 **File → Import → General → Existing Projects into Workspace**，指向该目录，选择工程 `test1`。保持使用现有路径，选择 **Debug** 配置。重新全量编译时，在 **Project → Clean** 中只选 `test1`，然后构建该工程。调试配置为 `test1 Debug`，目标文件为本工程的 [Debug/test1.elf](Debug/test1.elf)。烧录用的 [Debug/test1.hex](Debug/test1.hex) 来自同一次构建。

## 文件结构和变更

| 路径 | 用途和相对原工程的变化 |
|---|---|
| [code/test_mode/test_mode.c](code/test_mode/test_mode.c)、[test_mode.h](code/test_mode/test_mode.h) | 新增五态状态机、开环分档、直行和循迹 PI 调度、急停与人工复位 |
| [code/test_mode/test_mode_config.h](code/test_mode/test_mode_config.h) | 新增测试参数，包括两轮方向、分档、停顿、超时和闭环目标 |
| [code/test_mode/test_display.c](code/test_mode/test_display.c)、[test_display.h](code/test_mode/test_display.h) | 新增 IPS200 状态页 |
| [code/](code/) | 用最新 common_drivers 对应文件替换 `motor.c/h`、`encoder.c/h`、`steer.c/h`、`key.c/h`、`led.c/h`，新增 `driver_config.h`。这 11 个文件均与交付源逐字节一致 |
| [user/cpu0_main.c](user/cpu0_main.c)、[user/isr.c](user/isr.c) | 接入驱动初始化、前台显示和原有 PIT 中断。CPU1 入口、启动代码及中断配置沿用原工程 |
| [.project](.project)、[.cproject](.cproject) | 工程标识和输出路径改为 test1。在四个配置中将 `SMARTCAR_UI_KEY_READER` 限定于 `ui.c`，编入本工程的 `vision_track.c` 并排除旧的 `car_control.c` |
| [.settings/test1-debug-includes.opt](.settings/test1-debug-includes.opt) | 为 Debug 固定工程相对头文件搜索路径。ADS 生成命令时偶尔遗漏自己的 include 参数，独立响应文件使全量编译可复现 |
| [test1 Debug.launch](test1%20Debug.launch)、[.ads/winIDEAWorkspaces/Debug/ADS.xjrf](.ads/winIDEAWorkspaces/Debug/ADS.xjrf) | ADS 和 winIDEA 调试目标均指向本工程 ELF |
| [libraries/](libraries/)、[链接脚本](Lcf_Tasking_Tricore_Tc.lsl)、[code/speed_control.c](code/speed_control.c)、[code/smartcar_param.h](code/smartcar_param.h) | 沿用工程内的逐飞库、启动与链接配置、原速度 PI 和参数。原车控与 UI 源码也保留在 code 目录 |
| [code/vision_track.c](code/vision_track.c)、[vision_track.h](code/vision_track.h) | 复用三行二值化和加权中线，发布包含采集时刻的视觉结果。VSYNC/DMA5 和前台分工管理单缓冲 |
| [Debug/](Debug/)、[构建日志](build-report/ADS-Debug.log) | 本工程独立生成的二进制、映射文件和全量编译记录。原工程的 Debug 缓存、旧调试启动项和临时备份未搬入 |
| [README.md](README.md)、[实车测试清单](实车测试清单.md) | 工程操作说明与待签收项目 |

## 上电、调度与显示

1. CPU0 依次调用 `clock_init()`、`debug_init()`、`motor_init()`、`encoder_init()`、`steer_init()`、`key_input_init()`、`led_init()`、`test_mode_init()`。最后一步初始化已有速度控制器，并将双轮输出置零、舵机回中。
2. 初始化 SPI 方式的 IPS200，再调用 `vision_track_init()` 初始化 MT9V03X，随后绘制测试页，经过原有双核就绪同步，再以 `pit_ms_init()` 启动 CCU60_CH0 和 CCU61_CH0。两个通道均为 10 ms。`ENCODER_SAMPLE_MS` 和 `KEY_SCAN_PERIOD_MS` 表示驱动计算所依赖的周期，实际调度在 CPU0 入口配置。
3. CCU60_CH0 清中断标志，只调用一次 `update_speed_and_distance()`，随后运行 `test_mode_tick_10ms()`。初始化后，电机指令只从这条路径发出。直行闭环和获得视觉许可的循迹状态共用原有 `speed_control_update_10ms()`，每个节拍最多调用一次。
4. CCU61_CH0 清中断标志，只调用一次 `key_Handle()`。扫描、消抖、发布事件和锁存在按键驱动内完成。前台 `test_mode_poll()` 先检查急停，再从 `KEY_READER_CONTROL` 读取事件并提交模式请求。
5. 前台先轮询按键和急停，再调用 `vision_track_task()` 处理一帧。约每 200 ms 取一次一致的状态快照，退出临界区后刷新屏幕。显示左右轮请求 PWM、方向修正后的有符号 PWM、最近一个 10 ms 周期的有符号脉冲、实测速度、最终目标和斜坡参考速度。速度单位均为 m/s。屏幕刷新、图像处理和串口打印均不占用 10 ms 控制中断。

## 按键和状态

| 入口 | 状态 | 电机、速度控制器和退出条件 |
|---|---|---|
| 上电、开环超时或急停人工复位成功 | `TEST_IDLE` | 双轮 PWM 为零，舵机回中，速度控制器停止。等待新的模式键 |
| KEY2 稳定松开 | `TEST_MOTOR_OPEN_LOOP` | 先显示 `WAIT 0`，满足停稳条件后进入 `RUN`，两轮按各自配置逐级输出。到时自动回到 IDLE |
| KEY3 稳定松开 | `TEST_SPEED_CLOSED_LOOP` | 先归零、清积分并等待停稳，再启动原速度 PI 和原斜坡。KEY1 可急停。KEY4 切换到循迹时先归零并等待放行 |
| KEY4 稳定松开 | `TEST_VISION_TRACK` | 先显示 `VISION TRACK / WAIT 0`，待停稳且有入场后采集的新鲜有效帧才进入 RUN。再次按下并松开 KEY4 回到 IDLE |
| KEY1 连续三次采到按下，或发生严重故障 | `TEST_FAULT` | 停止速度控制器、双轮归零、舵机回中，LED4 每 200 ms 翻转一次。普通模式键不能解除 |

新模式在控制节拍中生效。每次切换先清积分、双轮归零。进入任一驱动模式都需要同时满足 **至少 300 ms 零输出停顿**和**连续 200 ms 两轮输出、编码器增量均为零**。等待达 2 s 仍未放行时，进入 `COAST TIME` 故障。该保护也用于反向开环切换到正向闭环。重复按 KEY2 或 KEY3 不会重启对应模式。视觉模式中的 KEY4 用于正常退出。

## 开环架空测试

1. 固定车身，架空两侧驱动轮，准备可直接切断电机供电的实体开关。上电检查 `IDLE / STOPPED`、`ESTOP CLEAR`、双轮 PWM 为零、舵机回中。
2. 在 [test_mode_config.h](code/test_mode/test_mode_config.h) 设置 `TEST_LEFT_DIRECTION`、`TEST_RIGHT_DIRECTION`。两者依次取 `(+1, 0)` 可单测左轮，`(0, -1)` 可单测右轮反转，`(+1, +1)` 为交付默认的双轮逻辑正转。修改配置后重新编译，先确认车辆停稳再更换固件。
3. 每轮均有独立的 `TEST_LEFT_PWM_START/STEP/MAX` 或 `TEST_RIGHT_PWM_START/STEP/MAX`。默认从 300 起，每 500 ms 增加 100，最大 1200，到顶保持。占空比单位为万分之一，300 和 1200 分别对应 3% 和 12%。这些数值用于首轮架空验证。
4. 按下并松开 KEY2，观察 `WAIT 0 → RUN`。记录左右轮起转档、旋向、脉冲符号和速度。屏幕 `PWM req` 是上层逻辑请求，`PWM apply` 是限幅并按现有左右轮方向修正后的有符号指令。两个电机的 `MOTOR_*_REVERSE` 当前均为 1，EN 端 PWM 幅值始终非负。
5. 自进入开环起累计 10 s（包含入场停顿）即回到 IDLE。提前停车按住 KEY1。KEY4 会切换到可驱动车辆的循迹模式。轮子仍在转动时，先观察零输出及停稳，随后再启动相反方向测试。

## 速度闭环测试

先用单轮开环确认，逻辑前进时该轮的实测速度为正。若符号相反，记录轮侧与接线，结束测试。反馈方向正确后按下并松开 KEY3。完成停顿后，原 PI 从零积分启动。两轮共用 `TEST_TARGET_MPS = 0.20`，原速度斜坡为 0.20 m/s²，因此每个 10 ms 控制节拍参考速度增加 0.002 m/s。

比较屏幕左右两列的 `SET m/s`、`RAMP m/s`、`V m/s`、`PWM req` 和 `PWM apply`，记录超调、稳定速度和左右差异。闭环持续运行至模式切换或急停。按住 KEY1 停车。切换至 KEY2 开环或 KEY4 循迹会先停止旧控制器、归零并重新执行停稳等待。

## KEY4 基础循迹

**最简上车流程**

1. 架空驱动轮并准备实体断电开关。上电保持 IDLE，把赛道分别置于画面中间、左侧和右侧。在 IPS200 上确认 `FR` 持续增加、`CAM` 显示 `TRACK OK`。三条小图分别对应近、中、远行，红标是边界，绿标是该行中点。无边界时该行数值为 −1。
2. 车道中心向图像右侧移动时，`ERR` 应为正，向左应为负。先独立确认图像方向，再验证舵机转向。要在电机零输出下实测转向，可临时把 `TEST_VISION_TARGET_MPS` 设为 0，重新构建、烧录，按 KEY4 进入仅转向测试。默认正误差使 `SERVO` 减小，轮子应向车道修正。若机械方向相反，检查 `VISION_STEER_SIGN`。屏显 SERVO 为本次实际下发的占空比指令。
3. 用 KEY2 验证两轮反馈符号，用 KEY3 确认速度闭环确实可用。首次循迹给定 **0.12 m/s**，项目内尚无该值的实车测速记录。确认第 2 步和速度方向后，再用非零目标固件架空试跑。按下并松开 KEY4，观察 `WAIT 0 → RUN`、舵机和两轮实测速度。按住 KEY1 检查停车，复位流程见下节。
4. 架空时遮挡镜头，确认丢线能停车。单独在断电后准备无摄像头工况检查初始化失败，帧中断测试在确保电气连接安全的条件下进行。各项通过后，在有人可立即断电的短直道、缓弯依次落地验证。循迹时再次按下并松开 KEY4 为正常退出。

**摄像头与控制路径**

- 逐飞 V3.4.5 的 `mt9v03x_init()` 返回 0 表示配置成功。库先尝试 SCCB，必要时回退 UART1。灰度图为 188×120，默认配置 50 fps，单个 `mt9v03x_image` 缓冲。实际帧率应在车上测量。
- P02.0 的 VSYNC 下降沿进入已有 ERU3 中断，记录起帧序号和时间后调用一次 SDK 回调。P02.1 的 PCLK 上升沿经 ERU2 触发 DMA5。22,560 字节分两段，每段 11,280 字节。DMA5 中断先调用 SDK 回调。末段完成标志置位且 DMA 已禁用后，前台才取得完整帧。分段中断、跨越新 VSYNC 的残帧以及 SDK 报错后中止的传输均不交付。
- 前台处理时屏蔽下一次 VSYNC 启动 DMA 的许可，10 ms PIT 仍可执行。完成三行扫描后，在短临界区发布结果并交还缓冲。来不及接收的 VSYNC 跳过。每帧只处理一次，屏幕使用前台保存的三条二值行。
- 入场时停止速度控制器、归零、舵机回中，同时记录已开始采集的帧序号。只有入场后才开始采集、至少两行有效且未超时的新帧能放行。控制中断按已有速度斜坡运行原 PI，并应用限幅后的舵机指令。KEY2 和 KEY3 仍可独立测试底盘。
- 运行中收到一帧无效结果时，下个 10 ms 控制节拍停车并回中。单帧干扰后取得新鲜有效帧才可从零斜坡恢复。连续两帧无效锁定 `LINE LOST`，即使两次控制节拍之间又收到有效帧也会停车。采集时间距当前达到 300 ms 则锁定 `FRAME TIME`。入场后一直有帧却在 1200 ms 内找不到可用赛道则锁定 `LINE LOST`。初始化失败时 KEY4 进入 `CAM INIT`。上述视觉故障需排除原因后人工重启 MCU。

**现场优先确认的参数**

| 参数位置 | 交付值及验证方法 |
|---|---|
| [视觉参数](code/smartcar_param.h) | `VISION_WHITE_TRACK=1` 表示所跟踪区域为亮色，黑色区域用 0。固定阈值 120，现场光照下核实直道和弯道两侧均有边界 |
| 扫描行与权重 | 近、中、远 y 为 110、90、65，权重 3、2、1，至少两行有效。每行接受不触碰画面边缘、宽 8～184 像素的连续区域。按真实镜头俯角、赛道宽度和弯道视野核实 |
| 转向 | 误差为加权中线减图像中心 94，单位像素。默认 `VISION_STEER_SIGN=-1`，`VISION_STEER_KP=2.0` 占空比单位/像素，限幅 ±140，中值 1505。先验方向再调整增益 |
| [测试参数](code/test_mode/test_mode_config.h) | 循迹目标 0.12 m/s，当前配置上限 0.20 m/s，目标 0 可单测转向。确认轮速反馈和低速 PI 能稳定带动车辆后再上地 |

## 急停和人工复位

1. 在任一状态按住 KEY1。第三次连续低电平采样即锁存停车请求，不需要松手。连续保持按下只产生一次通知。前台优先读取独立停车通知，控制中断也独立检查锁存，即使前台正在刷新屏幕，双轮最迟在锁存后的下一次控制节拍收到零指令。屏显更新可能稍晚于 PWM 归零，机械停稳时间需另测。
2. 确认屏幕出现 `FAULT / KEY1`、`ESTOP LATCHED`，并确认双轮已物理停稳。松开全部四键。控制侧须已收到停车通知。两轮施加 PWM 和脉冲均连续 200 ms 为零，期间四键释放，屏幕才会显示 `RESET YES`。
3. 在 ADS 调试器的 **Expressions / Watch** 中对 `test_manual_reset_request` **单次写入 1**，让 CPU 运行处理。底层还会核对四键原始电平、稳定态和消抖计数。过早的请求会被消费，需要满足条件后重新写入。
4. 复位后确认 `ESTOP CLEAR`、`IDLE / STOPPED` 和双轮 PWM 为零。只有重新按下并松开模式键才会启动。单纯松开 KEY1、重复按模式键或积压的旧事件均不能重启车辆。
5. `COAST TIME`（等待停稳超时）、`CONFIG`（闭环目标越界）和 `REPORTED`（CPU0 调用 `test_mode_request_fault()`）属于持续故障。排除原因后人工重启 MCU，重新从 IDLE 开始。软件急停需要 MCU 持续运行，实车另备实体断电开关。

## 验证记录

- 使用 ADS 的 TASKING Debug 配置，目标 TC264D / tc26xb，C99。从空 Debug 输出目录完整构建，228 个不同 C 源文件各编译一次，成功链接并生成 ELF、HEX 和 MAP。结果为 **0 错误、26 警告**。26 条均为沿用的 `ui.c` 中 `W523` 指针类型警告。ROM 为 78,968 字节，RAM 为 49,590 字节。[完整构建日志](build-report/ADS-Debug.log)
- 五个硬件驱动各编译一份，`SMARTCAR_UI_KEY_READER` 只出现在 `ui.c` 的编译命令中。检查生成依赖和构建参数，源码和驱动头均来自 test1 工程树。原 `smartcar` 的 1,453 个文件在操作前后逐项 SHA-256 一致。
- 本次用实际视觉、驱动、PI 和状态机代码，配合合成灰度帧与模拟外设执行主机联测。白色赛道默认配置，以及黑色赛道、反转舵机符号和零目标的组合配置，各通过 9 组场景。覆盖三行边界、加权误差、转向限幅、分段与残帧、入场旧帧、延迟处理、初始化失败、连续丢线、无帧超时、模式切换及长按急停。真实画面、帧率、伺服和车速响应、停车距离及全程表现待[实车测试](实车测试清单.md)。

本次相对底盘测试版修改了 `.cproject`、`user/cpu0_main.c`、`user/isr.c`、`code/vision_track.c/h`、`code/smartcar_param.h`、`code/test_mode/test_mode.c/h`、`code/test_mode/test_mode_config.h` 和 `code/test_mode/test_display.c`。同步更新本说明、实车清单、工作区 README 入口和构建日志，并重新生成 Debug 产物。

| 本次生成物 | SHA-256 |
|---|---|
| [Debug/test1.elf](Debug/test1.elf) | `f7bbf0ac23a8f957749b13c9c629e8af6882ad6783e502d72c39d2bcdb7aa691` |
| [Debug/test1.hex](Debug/test1.hex) | `a89d130f457b9a19d7f252e3107c543b0856b0215584a85b427d126c164cfb04` |
