# ADASim

轻量级自动驾驶算法仿真平台。C++17 写仿真底座（物理、传感器、时间轴、渲染），
Python 写算法进程（感知、决策、控制），两者走自定义 TCP/UDP 协议，可单机、可跨机。

设计目标不是"仿真得像"，而是**让算法迭代和失效复现这两件事变快**：
任何一帧都能在毫秒级回溯到当时的完整世界状态，任何一次失效都能被重新跑出来。

- 十万点云 720p 渲染 P99 = 5.99 ms（实测，见 `docs/实验报告.md`）
- 任意目标帧回溯位姿偏差 0.000e+00（逐位一致，见 `tests/replay_check.cpp`）
- 断线兜底：算法进程失联后底座自行降速停车，不追尾

## 1 架构

四层，依赖严格单向。L3 领域层不依赖 Qt，由 `cmake/LayerGuard.cmake` 在配置期强制检查。

```
L1 presentation   Qt 窗口、视口、交互          src/presentation
L2 application    仿真主控、场景服务、实验编排   src/application
L3 domain         物理、传感器、时间轴、模型     src/domain       ← 纯 C++17
L4 infrastructure Qt 渲染、TCP/UDP 网关、JSON   src/infrastructure
```

L2 不认识 L4，L4 不认识 L2，二者只通过 L3 定义的端口交接，连接点在 `src/main.cpp`。
改一处实现类不需要动另一层——这是"能换掉渲染器而不动算法"的前提。

外挂的进程边界：

```
        ┌──────────────┐   TCP 45100 状态帧(50Hz)   ┌────────────────┐
        │  adasim 底座 │ ─────────────────────────→ │  python/run.py │
        │  (C++/Qt)    │ ←───────────────────────── │  感知/规划/控制 │
        └──────────────┘   TCP 45100 控制指令        └────────────────┘
               │           UDP 45101 点云(10Hz)
               └──────────────→
```

状态帧是"长度前缀 + JSON"，点云是分片二进制（每片 120 点，不超 MTU）。
算法进程挂掉、卡住、发垃圾，底座都不会被拖死：`StandbyGuard` 接管降速。

## 2 构建

依赖：CMake ≥ 3.20、C++17 编译器、Qt6 ≥ 6.2（仅 GUI/压测/批量评估需要）。
Python ≥ 3.10（算法进程与 AI 模块，仅跑 C++ 时可以不要）。

```bash
# 完整构建（含界面）
cmake -S . -B build/qt -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH=/path/to/Qt/6.x/gcc_64
cmake --build build/qt

# 只构建领域层与无界面闭环（无 Qt 也能编译，用于 CI 或纯算法回归）
cmake -S . -B build/core -DADASIM_WITH_GUI=OFF
cmake --build build/core
```

没有 Qt 时 `find_package` 会失败并自动关闭 GUI 目标，不会让整个构建红掉——
`adasim_domain`、`adasim_app`、`adasim_headless` 照常产出。

产物在 `build/qt/bin/`：

| 可执行 | 用途 |
|---|---|
| `adasim` | 带界面的仿真底座 |
| `adasim_headless` | 无界面闭环，跑确定性检查与吞吐基线 |
| `adasim_bench` | 点云/渲染压测，输出帧时间分位数 |
| `adasim_batch` | 常驻评估服务，行式 JSON 进、行式指标出（供 AI 搜索调用） |

## 3 跑起来

```bash
# 1. 起底座（需要 Qt 的 bin 在 PATH 里）
./build/qt/bin/adasim

# 2. 另开一个终端起算法进程
python python/run.py                 # 持续运行
python python/run.py --duration 20   # 跑 20 秒后退出并打印汇总
python python/run.py --truth         # 跳过感知，直接用真值（对照实验）
```

底座命令行：

```bash
adasim scenarios/curve_tracking.json          # 指定场景启动
adasim scenarios/sandbox.json                 # 无尽沙盒：空路面，右键注入目标
adasim --shot out.png --shot-delay 8000       # 跑 8 秒截图退出，用于无人值守取图
adasim --record out/frames --frames 16 --interval 700 \
       --metrics out/timeseries.csv           # 连续录帧 + 100 ms 采样（Demo 用）
```

算法进程汇总会给出：状态帧/指令帧数、平均点数、平均计算耗时、最小车距、
最小 TTC、决策分布，以及横向/航向/速度误差 RMS（`--warmup` 之后开始统计）。

### 3.0 无尽沙盒与运行时注障

工具栏点「沙盒」进入空路面（`scenarios/sandbox.json`，`openWorld`），**右键点视口任意位置**
就在那里注入一个虚拟目标：默认轿车，按住 Shift 是卡车、Ctrl 是行人。工具栏「清除注入」一键清空。

这套东西的要点不是"多画一个障碍"，而是**雷达欺骗**：注入的目标

- 会进激光雷达的点云（轮廓点与真实障碍物同一条采样公式、同一条距离衰减）；
- **不会**进 `StatePacket.obstacles`（真值列表），也不参与物理推进与碰撞。

于是算法只能从点云里把它聚类出来——测的是感知链路，而不是给它一个答案。算法在线时车会
减速停住；算法离线时兜底律看不到它（兜底按真值算车距），车会直接从虚线框上开过去。
后一种表现是设计使然，也正好是演示时最有说服力的一组对照。

无人值守时用命令行注同样的目标（与右键走同一条 `SimController::injectGhost`）：

```bash
adasim scenarios/sandbox.json --inject "50,0"             # 自车前方 50 m 注入一辆车
adasim scenarios/sandbox.json --inject "40,-3.5,truck" --inject-delay 2500   # 2.5 秒后注入到邻道
```

### 3.0.1 回放与 HUD

- 底部进度条按帧号取值，**拖拽即回看**、随时暂停。回看不销毁记录（`previewSeek`），
  按播放才从当前位置另起一支（`start()` 截断）。「回到最新」跳回录制末尾。
- 左上角 HUD 实时显示：车速（km/h）、帧率与光栅化耗时、往返时延与指令龄、
  算法在线状态与输出（`a`、`δ`、决策原因）、注入目标数与点云数。
  时延由算法进程回传（底座测不到对端处理耗时），带的是均值：它天然含一个帧周期
  （`ackSeq` 随状态帧回带），净时延（扣掉周期）在算法汇总里单独给出。


### 3.1 一条命令跑通 Demo

```bash
python tools/demo.py
```

默认跑**三段连播**，每段各录一次真实仿真，串成一条可拖的时间轴：

| 段 | 场景 | 演什么 |
|---|---|---|
| ① 跟驰 | `following_lead` | 算法在线，8 m/s 追近 4.2 m/s 前车，IDM 收敛到稳定间距 |
| ② 切入 | `cut_in_brake` | 算法在线，邻道车切入后开始减速，自车让速保距 |
| ③ 断链 | `cut_in_brake` | **同一场景**，算法进程接入 5 秒后退出，底座兜底接管 |

第三段刻意复用第二段的场景：唯一变量是算法进程死没死，两段对比才是可控实验。
产物 `output/demo/index.html` 是单文件（帧内联成 data URI），可直接发给别人：
三段标签跳转、自动连播、每段一张车速/车距曲线（带播放头与离线灰底）、
本段读数随段切换。画面上还有角标区分「算法未接入 / 离线兜底制动中 / 离线但车距有余量」——
不标的话，观众会把底座的刹车当成算法的功劳。

```bash
python tools/demo.py --only drop                 # 只跑第三段
python tools/demo.py --drop-at 4                 # 第三段 4 秒断链
python tools/demo.py --scenario scenarios/curve_tracking.json   # 退回单场景
python tools/demo.py --frames 20 --interval 500  # 录得更密
python tools/demo.py --rebuild                   # 复用上一轮的帧，只重排页面
```

时序固定是刻意的：手工演示要掐两次时刻（算法进程什么时候接入、什么时候退出），
掐错一次看到的就是"算法没连上"或"断链发生在录完之后"，而这两种情况的画面都很正常，
看不出问题。

## 4 场景

`scenarios/` 下 10 个 JSON 场景，索引见 `scenarios/README.md`。字段：

```jsonc
{
  "name": "跟驰",
  "durationSeconds": 40,
  "egoSpeed": 8.0,                       // 自车初速，缺省 0
  "egoPose": { "x": 0, "y": 0.3, "theta": 0 },   // 自车初始位姿，缺省原点朝 +x
  "laneWidth": 3.5,
  "friction": 0.9,                       // 路面附着系数，接进物理引擎的加速度下界
  "curvature": 0.0,                      // 1/m，正值左弯；随状态帧下发给算法
  "sensor": { "lidarRange": 60, "lidarPointsPerMeter": 1.2 },
  "obstacles": [
    { "x": 25, "y": 0, "theta": 0, "speed": 8,
      "behavior": { "kind": "car_following" } }
  ]
}
```

`behavior.kind` 支持 `constant` / `car_following` / `cut_in` / `brake`。
新增场景只需要写 JSON，不用改代码——这是"AI 生成场景"能落地的前提。

## 5 测试

```bash
cd build/qt && ctest --output-on-failure       # 全套，约 1 秒
ctest -L unit                                  # 只跑 C++ 用例
ctest -L python                                # 只跑算法与场景生成自检
```

6 个用例：

| 用例 | 守住什么 |
|---|---|
| `standby_guard_check` | 断线兜底连续律：介入次数、最小净距、对照组确实追尾 |
| `cut_in_check` | 切入几何（端点不横移、航向连续）、危险度、重推演一致 |
| `replay_check` | 19 个目标帧回溯与直跑逐位相等 |
| `headless_closed_loop` | 无界面闭环确定性，兼作"领域层没偷偷依赖 Qt"的守门员 |
| `algo_selftest` | 算法进程协议编解码、感知、规划离线自检 |
| `ai_selftest` | 场景生成 schema、模板、校验、NL 解析、GP 搜索 |

Python 用例要求解释器 ≥ 3.10（`dataclass(slots=True)` 的下限），CMake 会按这个版本
下限去探测；机器上装了多份且不在 PATH 里时，用 `-DADASIM_PYTHON_EXECUTABLE=<path>` 指定。

## 6 实验复现

```bash
export PATH="<Qt>/bin:<Qt>/Tools/mingw1310_64/bin:$PATH"

python tools/run_experiments.py --group e2    # 通信时延：轻载/重载/真值对照
python tools/run_experiments.py --group e4    # 闭环精度：6 个场景
python tools/eval_scenario_gen.py             # 场景生成有效性：50 条自然语言描述
./build/qt/bin/adasim_bench --no-udp          # 渲染性能：1 万~10 万点
```

原始数据落 `output/experiments/`，口径与结论见 `docs/实验报告.md`。

## 7 AI 增强模块

两块，都只依赖标准库（`ai/adasim_ai/`）：

**自然语言 → 场景**。三档降级：语言模型解析 → 规则解析 → 预置模板。
生成结果过 schema 校验、几何校验（含 0.1 s 步长的轨迹互穿检测）后才落盘。

```bash
python ai/scenario_gen.py "前车 25 米同速跟车，第 3 秒邻道车切入并刹车"
python ai/scenario_gen.py --check scenarios/cut_in_brake.json
```

**贝叶斯失效搜索**。自写 Matérn 5/2 高斯过程 + Cholesky + EI 采集函数，
超参在初始化时用边际似然选一次（每轮重选会把 100 次预算烧到 55 s，选一次降到 24 s）。
评估走 `adasim_batch` 常驻进程，避免每次 fork 一遍 Qt。

```bash
python ai/search.py --scenario ai/presets/cut_in_brake.json --budget 100
```

## 8 扩展点

| 想加什么 | 改哪里 | 要不要动别的层 |
|---|---|---|
| 新的参与者行为 | `BehaviorKind` 加枚举 → `ActorBehavior::advanceActors` 加分支 → `ScenarioService` 解析 | 不用 |
| 新的传感器 | `src/domain/sensor/` 加模型 → `WorldState` 挂字段 → 网关加一个通道 | 不用 |
| 新的协议字段 | `StatePacket` → `TcpUdpGateway::stateToJson` → `python/adasim_algo/protocol.py` | 三处，缺一处会静默退化 |
| 换渲染器 | 实现 `src/domain/port/` 里的渲染端口 | 不用 |
| 新的评价指标 | `ExperimentRunner::EvaluationConfig` / `ExperimentMetrics` | 不用 |

协议加字段是最容易漏的一类改动：C++ 端加了、Python 端没解析，程序不报错，
只是那个字段永远是默认值。弯道曲率就踩过这个坑（字段只解析不生效，弯道场景
静默退化成直道），所以每次动 `StatePacket` 都该同步跑一遍 `ctest -L python`。

## 9 已知边界

- 参与者只沿各自航向直线推进，不跟随道路曲率。弯道场景里只放静止或短时存在的车。
- 感知是简化实现：点云聚类 + 最近邻关联，没有分类网络，也没有误检建模。
- 十万点下算法侧点云重组（每帧 858 片）是净时延主因，不是链路。
- 场景生成的语言模型通路需要外部密钥，无密钥时自动降级到规则解析。

## 10 文档

| 文档 | 内容 |
|---|---|
| `docs/软件架构设计.md` | 分层依据、端口设计、线程模型、协议、回溯机制 |
| `docs/实验报告.md` | 五组实验的原始数据、统计口径、与开题指标的对照 |
| `docs/性能压测报告.md` | 点云规模扫描、帧时间分位数、瓶颈定位 |
| `docs/AI增强模块设计.md` | 场景生成 schema、校验策略、GP 搜索与加速 |
| `scenarios/README.md` | 场景索引：每个场景服务哪组实验、关键参数含义 |
