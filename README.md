# GTSAM 实战课程（lessons）

六节可以直接编译、直接运行的 GTSAM 小课（lesson0 是模板），每节一个独立可执行文件，
输出就是讲义：不打印一堆看不懂的数字，而是把**做了什么、为什么这样做、结论是什么**写在终端里。

**第一次来 → 先跑 `xmake run lesson0`**，它给出「一个自定义因子 + 一次优化」的最小骨架，
后面的课都是在这套骨架上加东西。

## 环境

| 依赖 | 说明 |
| --- | --- |
| GTSAM | 已安装在 `/usr/local`（含 `libgtsam`、`metis-gtsam`、`cephes-gtsam`） |
| Eigen | `add_requires("eigen <5")` |
| TBB | `add_requires("tbb", { system = true })` |
| xmake | 3.x |

`xmake.lua` 里的 `gtsam_deps` 规则统一处理 GTSAM 的 include/link/rpath/LD_LIBRARY_PATH，
所以每个 lesson target 只要 `add_rules("gtsam_deps")` + `add_packages("eigen")` 即可。

## 构建与运行

```bash
xmake build            # 编译全部 target
xmake run lesson0      # 运行第 0 课（模板，自动带上 LD_LIBRARY_PATH）
xmake run lesson1
xmake run lesson5
xmake build -r lesson5 # 只重编某一课
```

## 目录结构

```
src/
├── include/
│   ├── lesson_utils.hpp      # 雅可比自检、Pose2 误差、计时、章节标题
│   └── factors/
│       ├── range_factor.hpp      # RangeFactor（lesson1/2 用）
│       └── gps_point_factor.hpp   # GpsPointFactor（lesson2/4 用）
└── lessons/
    ├── lesson0_template.cpp          # 模板课（建议从这里开始）
    ├── lesson1_range_factor.cpp
    ├── lesson2_noise_robust.cpp
    ├── lesson3_pose_graph.cpp
    ├── lesson4_sensor_fusion.cpp
    └── lesson5_gnc_loop_closure.cpp
```

## 课程表

### Lesson 0 — 模板：从零写一个因子

`xmake run lesson0` · `src/lessons/lesson0_template.cpp`

| 章节 | 内容 |
| --- | --- |
| 1 | 一个因子的 4 个必备件（基类 / 构造 / `evaluateError` / `clone`）+ 残差怎么被白化 |
| 2 | 写完因子立刻做雅可比自检（三处 `[ OK ]`） |
| 3 | 搭图 → 给初值 → LM 求解；顺带演示「冗余边一致性检查」 |
| 4 | 边缘协方差：有先验的 P0（σ≈0.02）vs 只靠边推的 P1/P2（σ≈0.046） |
| 5 | 去掉先验 → Hessian 秩亏（nullity=2，全局平移）；GN 抛异常、LM 换个初值就换个「解」 |

### Lesson 1 — 自定义因子：一维测距因子

`xmake run lesson1` · `src/lessons/lesson1_range_factor.cpp`

| 章节 | 内容 |
| --- | --- |
| 1 | 手写雅可比 vs 中心差分数值雅可比（`lesson::checkJacobian`） |
| 2 | 故意写错雅可比 → 自检立刻失败，说明为什么必须自检 |
| 3 | 测距因子对姿态 θ **不可观**：Hessian 最小特征值 = 0，两个不同初值给出两个「最优解」 |
| 3b | 加先验"能解"，但那只是把先验复制了一遍 |
| 4 | 加一个 bearing 因子，θ 立刻变可观 |
| 5 | 残差大 ≠ 影响大：残差要乘上噪声模型的权重才有意义 |

### Lesson 2 — 噪声模型与鲁棒核

`xmake run lesson2` · `src/lessons/lesson2_noise_robust.cpp`

| 章节 | 内容 |
| --- | --- |
| 1 | 同一张图、同一组测量：Gaussian / Cauchy / Huber 三种噪声模型对比，并打印每个测量的等效权重 |
| 2 | Robust 核的代价非凸：从 `(0,0)` 出发的 Cauchy 会掉进错误局部极小；用 Huber 解做初值后误差 0.003 |
| 3 | Diagonal 噪声：每个轴独立的可信度直接决定融合结果 |

### Lesson 3 — 位姿图与 iSAM2

`xmake run lesson3` · `src/lessons/lesson3_pose_graph.cpp`

| 章节 | 内容 |
| --- | --- |
| 1 | Pose2 是流形：`compose / between / retract / localCoordinates`，为什么不能对 (x,y,θ) 直接线性插值 |
| 2 | 因子图的稀疏结构（邻接表） |
| 3 | 回环把累积漂移"拉回去"：加回环前后的逐位姿误差 |
| 4 | iSAM2 增量求解：只重算受影响的子树；最终结果与批量 LM 一致到 1e-5 |

### Lesson 4 — 里程计 + GPS 融合

`xmake run lesson4` · `src/lessons/lesson4_sensor_fusion.cpp`

| 章节 | 内容 |
| --- | --- |
| 1 | 各信息源单独使用：死推（漂移）、只用 GPS（朝向不可观） |
| 2 | 融合：普通最小二乘被 2 个 GPS 跳变拖到比死推还差；Cauchy 鲁棒融合后 RMSE 0.42 m |
| 3 | 边缘协方差：有 GPS 内点的位姿更确定；**跳变帧虽然有 GPS，方差反而最大**（有效信息 vs 有没有传感器） |

### Lesson 5 — 错误回环与 GNC

`xmake run lesson5` · `src/lessons/lesson5_gnc_loop_closure.cpp`

| 章节 | 内容 |
| --- | --- |
| 0 | 12 帧走廊（去程 + 回程），5 条回环里 2 条是**错误**的（偏 2~3 m） |
| 1 | 普通 LM / LM+Cauchy / GNC+TLS 对比：1.19 m → 0.13 m |
| 2 | 权重对比：Cauchy 软权重（内点≈1、外点≈0.12），GNC 硬权重（0/1），可直接当外点分类器 |
| 3 | 逐帧误差：外点如何把整条闭环拉"斜" |
| 4 | 4 个方向各异、只偏 0.5 m 的"隐蔽"外点：GNC 分类 6/7 |
| 5 | 4 个**彼此自洽**（共用同一偏置）的外点：GNC 把它们全收下、把真回环全丢掉；Cauchy 反而稳 → 因此需要 PCM 之类的前端 |

## 动手练习

每节课文件顶部的注释里都留了 2~3 个 TODO，改完直接 `xmake run lessonN` 就能看到差别。例如：

* Lesson 0：把 2 维位移残差换成 1 维距离残差，第 5 节的零特征值会变多；给 P1 也加先验。
* Lesson 2：把 Cauchy 的 `k` 改成 0.3 / 5.0；把 GPS 噪声换成带相关性的 `Gaussian::Covariance`。
* Lesson 3：回环加 `Robust(Cauchy)` 后人为偏 0.5 m，打印权重看它是否被降权。
* Lesson 4：GPS 噪声换成各向异性 `Diagonal(0.1, 0.8)`；把 GPS 频率从每 3 帧降到每 6 帧。
* Lesson 5：去掉 `setKnownInliers`；把 `lossType` 换成 `GMC`；用 `setInlierCostThresholds` 手动改内点阈值。

## 调试小抄

* `lesson::checkJacobian(factor, values)` —— 手写雅可比的数值自检（相对误差 > 1e-5 即失败）。
  传进去的 `values` 必须**包含该因子用到的所有变量**，否则会抛 `ValuesKeyDoesNotExist`。
* `lesson::printFactorErrors(graph, values)` —— 逐因子打印白化误差，快速定位坏因子。
* `graph.error(values)` —— 注意：**不同噪声模型下的 cost 不可直接比较**（鲁棒核的 cost 是有界的）。
* `Marginals(graph, values).marginalCovariance(key)` —— 边缘协方差；对角线开方就是 sigma。
* Robust 权重：`std::dynamic_pointer_cast<noiseModel::Robust>(factor->noiseModel())->weight(factor->unwhitenedError(values))`
  （传**未白化**残差，`Robust::weight` 内部会用内层高斯算马氏距离）。