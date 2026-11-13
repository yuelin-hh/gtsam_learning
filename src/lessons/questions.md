# 因子图秩亏是什么意思

## 一句话

**秩亏（rank deficiency）= 因子图的雅可比矩阵“喂不满”全部未知量的自由度**：存在一组非零的状态改变量 δx，使得**所有因子的残差都不变**（J·δx = 0）。这组 δx 叫做**零空间**（null space），也就是**不可观方向**（unobservable / gauge freedom）。在这种方向上，代价函数完全平坦，因此有无穷多个“最优解”。

## 数学表述

把图线性化后，GTSAM 交给你的是一次白化过的线性最小二乘：

```
min_δx  ‖ J·δx - b ‖² ,     信息矩阵（Hessian）Λ = JᵀJ
```

- J 的行数为 `Σ 各因子残差维数`，列数为 `Σ 各变量切空间维数`（即“总自由度” N）。
- **满秩**：rank(J) = N，则 Λ 可逆，解唯一。
- **秩亏**：rank(J) = N − k，则 Λ 有 k 个零特征值，Λ⁻¹ 不存在。

注意，秩是 **J 的列空间维数**，与因子个数无关：因子再多，如果约束方向都一样（彼此冗余），秩也不会增加。所以“因子很多”并不等于“定得下来”。

## 用 lesson0 的真实数字算一遍

3 个 `Point2` → N = 3×2 = **6**；3 个位移观测，每个 2 维残差 → J 是 6×6。看起来正好，可是：

```
Hessian 维度 = 6x6，零特征值个数 = 2
特征值 = -1.6e-13  0  1200  1200  1200  1200
```

原因很直观：位移观测只约束**相对位置**。把整张图在 x、y 方向同时平移任意量，所有 `p_j - p_i` 都不变 → 残差全不变 → 有 **2 个零方向**（x/y 全局平移），rank = 4。

这就是 lesson0 第 5 节那两个解的来历：

```
初值 P0=( 0.000, 0.000) → 解 P0=(-0.007, 0.024),  P1-P0=(0.992, 0.558)
初值 P0=( 5.000,-3.000) → 解 P0=( 4.993,-2.976),  P1-P0=(0.992, 0.558)   ← 形状完全一样
两条解的 error = 0.789758 vs 0.789758（相同）
```

**可观的部分（形状）由观测决定，不可观的部分（绝对位置）由初值决定。** 这正是“秩亏”在工程上的真实含义：解里有一块是瞎猜的。

Lesson 1 是同一个现象的另一个例子：**测距因子对姿态 θ 不可观**。距离和机器人朝哪看无关，所以 H 的 θ 方向特征值为 0；估计能“收敛”完全是因为先验，而 θ 的边缘方差恰好等于先验方差 0.25 = σ²，说明 θ 的信息 100% 来自先验、0% 来自测距。

## 在 GTSAM 里会怎么暴露

| 现象 | 原因 |
| --- | --- |
| `GaussNewtonOptimizer` 抛 `IndeterminantLinearSystemException`，并指出“near variable p0” | GN 直接解 Λδx = b，Λ 奇异就失败 |
| `LevenbergMarquardtOptimizer` **不报错**、还能“收敛” | LM 解的是 (Λ+λI)δx = b，阻尼把奇异方向糊住了；结果是初值依赖的假解 |
| `Marginals` 抛 `IndeterminantLinearSystemException`，或某个 σ 巨大甚至 inf | 边缘协方差要 Λ⁻¹，不可观方向没有有限方差 |
| 特征值谱里出现 ~1e-13 或更小（相对最大特征值） | 秩亏或接近秩亏 |
| 换初值/换随机种子，解就变 | 零空间分量没被数据约束 |

## 常见成因

1. **没有绝对参考**：只有相对观测（里程计、位移、between），整张图可以整体漂走。
2. **传感器本身缺自由度**：测距/距离观测没有方向；单目没有尺度。
3. **几何退化**：路标全部共线（三角化无深度）、机器人直线运动+单目、平面场景解 3D 位姿。
4. **图断开**：两个互不相连的子图，其中一块没先验 → 那一块整块自由（稀疏分解时会退化成两段求解）。
5. **参数冗余**：两个变量只以“和/差”的形式出现（比如 `a` 和 `b` 只以 `a+b` 进因子），单看一个就不可观。⚠️ 注意这跟“病态”不同：秩亏是**严格**的零方向；病态（ill-conditioned）是特征值极小但不为零（量纲差 1e6、噪声标准差写错 1000 倍）。工程上只能用阈值区分，所以 lesson0 才用 `|λ| < 1e-9·λmax` 来数零特征值。

## 怎么修

- **钉住 gauge**：给第一个位姿/点加强先验（`PriorFactor`，σ 要小到真正起作用）；或者子图每个连通块至少一个。这是最常用、最该先想到的做法。
- **补信息**：加一个与不可观方向**正交**的观测（lesson 1 里加 bearing 因子 → θ 立刻可观）。
- **参数化改造**：把不可观/冗余的参数消掉（固定一端、用相对量、把 `a,b` 重参数化成 `a+b, a-b`）。
- **只做诊断时**：用 LM 跑一遍、看 `Marginals` 会不会炸、看 `graph.linearize(v)->hessian().first` 的特征值谱，比盯着 cost 下降靠谱得多。
- **需要“能解”但不唯一时**：求最小范数解（伪逆 / 加极小 Tikhonov 正则 ρ‖δx‖²），并明确告诉下游“这块是不可观的”。

## 自查口诀

```
rank(J) 是否 == Σ 变量切空间维数？
零特征值个数 k == 不可观（gauge）自由度个数
解随初值变化的那几个分量，就是零空间方向
```

对应到实际系统：单目 SLAM 的 gauge 是 **7 维**（3 平移 + 3 旋转 + 1 尺度）；VIO 里重力方向让 roll/pitch 可观、IMU 让尺度可观，剩下 **4 维**（3 平移 + yaw）由“第一个位姿先验”钉住。这也解释了为什么 VIO 的初始化必须先把重力对齐、尺度估出来，否则那张图就是秩亏的。

代码里想直接验证，可以在任意 lesson 里加这几行（lesson0 的写法）：

```cpp
const auto linear = graph.linearize(values);          // 在当前线性化点
const Matrix H = linear->hessian().first;             // Λ = JᵀJ（已白化）
Eigen::SelfAdjointEigenSolver<Matrix> eig(H);
size_t nullity = (eig.eigenvalues().array() < 1e-9 * eig.eigenvalues().maxCoeff()).count();
```

# 每个因子在初始化时都会经过怎样的步骤，它的残差是怎么算的，雅可比矩阵又有什么用

## 从零理解：一个因子从被创建到被优化的全过程

先给一句话结论，后面逐层拆开：

> **因子本身不存任何状态值**。它只存三样东西：**它管哪几个变量（key）**、**测量值 z**、**测量有多准（噪声模型 Σ）**。
> 真正的“数值计算”发生在 GTSAM 拿着当前的 `Values` 来问它的时候 —— 先要**残差 e**，再要**雅可比 J**，然后拼成一个巨大的线性最小二乘问题去解。

---

## 0. 大图景：我们在解什么问题

一张因子图只有两种东西：

| 概念 | 代码里 | 含义 |
| --- | --- | --- |
| **变量** x | `Values`，`insert(P(0), Point2(1,0.6))` | 未知数（位姿、路标、标定参数…） |
| **因子** f_i | `OffsetFactor` / `RangeFactor` / `BetweenFactor` | 一条“观测”或“先验”，把几个变量绑在一起 |
| **残差** e_i | 你的 `evaluateError()` 返回的东西 | “按当前估计，这条观测本该是多少” 与 “实际测到多少” 之差 |

优化的目标函数就是所有因子残差（按可信度加权）的平方和：

$$
\min_x \sum_i \tfrac12 \|e_i(x)\|^2_{\Sigma_i}
\quad\text{其中}\quad e_i(x) = h_i(x) - z_i
$$

> $h_i(x)$ 是传感器的**正向模型**（预测），$z_i$ 是**实测**。比如“预测两点间距 = 1.0 m，实测 1.05 m”，残差就是 −0.05 m。
>
> 物理直觉：把每个变量想成一个**珠子**，每个因子是一根**橡皮筋**。残差 = 橡皮筋被拉长了多少；噪声 σ = 橡皮筋的“粗细”（σ 小 → 橡皮筋很硬，几乎不许拉长）。优化 = 松手，让所有橡皮筋的弹性势能之和最小。

---

## 1. 一个因子的完整生命周期（6 步）

### Step 0（编译期）：类声明决定了“形状”

```cpp
class OffsetFactor
    : public gtsam::NoiseModelFactorT<gtsam::Vector2, gtsam::Point2, gtsam::Point2> {
```

这行模板参数就是全部“元数据”：

- `Vector2` → **残差是 2 维**（所以约束 2 个自由度）
- `Point2, Point2` → **它管 2 个变量，类型都是 Point2**（每个 2 维）

GTSAM 靠这些信息知道“这个因子线性化后是 2 行、几列”，后面拼稀疏矩阵全靠它。**写错类型会在编译期或线性化时立刻炸**，这是好事。

### Step 1（构造）：只登记，不算数

```cpp
OffsetFactor(Key fromKey, Key toKey, const Vector2& measured,
             const SharedNoiseModel& noiseModel)
    : Base(noiseModel, fromKey, toKey), measured_(measured) {}
```

> **参数顺序为什么是 `(keys…, measured, noiseModel)`**：这是官方所有因子的统一约定
> （`gtsam::BetweenFactor` / `PriorFactor` / `RangeFactor` / `GPSFactor` 全都如此）。
> 读起来就是一句数学式子 `f(x1, x2; z, Σ)`：变量、测量、测量的可信度。
> 另外 `noiseModel` 是唯一可以带默认值的参数，而 C++ 要求带默认值的参数只能排在最后。
> **注意基类 `NoiseModelFactorT` 的顺序是反的**（`noiseModel` 在第一个），别被它带偏。

**这一步没有任何数值计算**，只有三件事：

1. `Key` 是什么？——GTSAM 把 `('p', 0)` 打包成一个 64 位整数（**高 8 位存字符、低 56 位存下标**）：
   `P(0) = Symbol('p', 0) = 0x7000000000000000`，`P(1) = 0x7000000000000001`。
   想拆回两部分用 `symbolChr(key)` / `symbolIndex(key)`，不要自己移位。`P(0)`、`P(1)` 就是两个 key。
2. `noiseModel` 是一个 `shared_ptr`，**被多个因子共享**（很省内存）。
3. `measured_` 是你的测量，**按值拷贝**进去，之后不会变。

> 这是初学者最容易误解的地方：**构造时没法算残差**，因为残差需要“当前估计值”，而此刻状态还没进来。
> 你的初值 `Values` 是另一回事（`Values initial; initial.insert(P(0), ...)`），因子只是登记了“我要读 `P(0)` 这个键”。

### Step 2：放进图里

```cpp
graph.emplace_shared<OffsetFactor>(P(i), P(j), z, noiseModel);
```

`emplace_shared` = `new` + 包成 `shared_ptr` + `graph.push_back()`。图里存的是一个**因子指针数组**（有序！顺序会影响后面打印的下标）。图此时还是“空壳”，一个数都没算。

### Step 3：算残差 —— `graph.error(values)` / `factor.error(values)`

调用链（这是 GTSAM 里真实存在的层次，名字都能在头文件里查到）：

```
factor.error(values)                        // ① 我要一个标量代价
  └─ noiseModel->loss(                      // ② 噪声模型决定怎么“罚”
        noiseModel->squaredMahalanobisDistance(
            factor.unwhitenedError(values)  // ③ 未白化残差 e = h(x) - z
        ))
```

而 `unwhitenedError(values)` 做的是（基类里已经写好，你不用管）：

```
values.at<Point2>(key0)  →  p_from     // ④ 从 Values 里按键取值
values.at<Point2>(key1)  →  p_to
evaluateError(p_from, p_to)             // ⑤ 你写的那一行公式
```

所以**你自己真正要写的只有一个公式**。三个“变换”被 GTSAM 包掉了：

| 层次 | 公式 | 单位 | OffsetFactor 的数值（σ=0.05） |
| --- | --- | --- | --- |
| 未白化残差 `unwhitenedError` | $e = h(x)-z$ | 米 | $(-0.026,\ 1.428)$ |
| 白化 `whiten(e)` | $R e,\ R=\Sigma^{-1/2}$ | 无量纲（σ 的倍数） | $(-0.515,\ 28.55)$ |
| 代价 `error` | $\tfrac12 \|Re\|^2 = \tfrac12 \sum (e_k/\sigma_k)^2$ | 标量 | $407.7$ |

**为什么必须白化？** 因为要把“米”和“度”、“米”和“像素”换成同一种货币 —— “相当于几个 σ”。这样 2 m 的 GPS 误差（σ=0.3）和 2 m 的里程计误差（σ=0.02）才能相加。这也是 Lesson 1 第 5 节“残差大 ≠ 影响大”的根源：

```
#0 keys=[x0] error=12.4957      ← 先验残差 5 m，但 σ=1 m
#4 keys=[x0,l1] error=0.00007   ← 测距残差 0.01 m，但 σ=0.01 m ⇒ 权重是 1e4 倍
```

`½` 是哪来的？高斯负对数似然：$-\log p(e) = \frac12 e^T\Sigma^{-1}e + \text{const}$。所以**最小化平方和马氏距离 = 最大似然估计**，不是随便定的。

### Step 4：线性化 —— `factor.linearize(values)`

优化器不是直接对“曲线”求解，而是在当前点做**一次泰勒展开**，再解线性问题。这一步里 GTSAM **第二次**调用你的 `evaluateError`，但这次带上“请把雅可比也给我”的请求：

```
std::vector<Matrix> H(2);                     // 输出参数：每个变量的雅可比
Vector e = factor.unwhitenedError(values, &H);  // 内部 → evaluateError(p_from, p_to, &H[0], &H[1])
noiseModel->WhitenSystem(H, e);                 // 白化：A = R·H,  b = R·(-e)
return JacobianFactor(keys, A, b);              // 变成“线性因子”
```

GTSAM 官方注释写得非常清楚：

$$
A\,\delta x - b \;\approx\; h(x\boxplus\delta x) - z
\qquad\Longrightarrow\qquad b = z - h(x) = -e(x)
$$

也就是说，线性化后每个因子贡献的是一个**线性方程**：

$$
\underbrace{R J}_{\text{A}}\ \delta x \;\approx\; \underbrace{-R e}_{b}
$$

### Step 5：拼成一张大稀疏矩阵，求解

所有因子的小 `JacobianFactor` 被摞成全局的 $A$（行数 = 所有残差维数之和，列数 = 所有变量自由度之和）和 $b$：

$$
\min_{\delta x} \|A\,\delta x - b\|^2
\;\;\Longrightarrow\;\;
\underbrace{A^TA}_{\text{信息矩阵 }\Lambda}\delta x = A^T b
$$

这一步就是**正规方程**（normal equations）。$A$ 极度稀疏（每个因子只碰几个变量），所以 GTSAM 用 QR / Cholesky 稀疏分解（Lesson 3 的“填充”就是在讲这件事），iSAM2 更进一步只重算被改动的子树。

### Step 6：更新状态，重复

$$
x \leftarrow x \boxplus \delta x
$$

- 普通向量：直接加。
- 流形（`Pose2`、`Rot3`）：用 `retract()`，即 $T \leftarrow T \cdot \text{Exp}(\delta)$（Lesson 3 第 1 节）。

然后回到 Step 3 重新算残差 → Step 4 重新线性化 → …… 直到 $\|\delta x\|$ 很小（GN）或代价不再下降（LM）。

### 一张表总结

| 阶段 | 谁调用 | 走到你的代码的哪个函数 | 输出 | 好比你 |
| --- | --- | --- | --- | --- |
| 构造 | 你 | 构造函数 | 一个只含 key/z/Σ 的空壳 | 登记“我要管谁” |
| 加图 | 你 | — | 图里多一个指针 | 挂上橡皮筋 |
| 算代价 | `graph.error(v)` | `evaluateError(x…)` （无 H） | 一个标量（用于打印/判断收敛） | 量橡皮筋拉多长 |
| 线性化 | 优化器每次迭代 | `evaluateError(x…, &H1, &H2)` | 残差 + 雅可比 | 量“往哪边推最省力、多省力” |
| 求解 | 优化器 | — | $\delta x$ | 松手 |
| 更新 | 优化器 | — | 新 `Values` | 珠子移动 |

---

## 2. 残差到底怎么算：手算两个真实例子

### 例 1：`OffsetFactor`（2 维向量残差，lesson0）

$$\begin{aligned}
h(x) &= p_{\text{to}} - p_{\text{from}} \\
e &= h(x) - z = (p_{\text{to}} - p_{\text{from}}) - z
\end{aligned}$$

取 `p_from=(0,0)`、`p_to=(1,2)`、`z=(1.0258, 0.5725)`：

$$
e = \begin{pmatrix}1-0\\2-0\end{pmatrix} - \begin{pmatrix}1.0258\\0.5725\end{pmatrix}
  = \begin{pmatrix}-0.0258\\1.4275\end{pmatrix}
$$

**逐分量**理解：x 方向吻合得很好（残差 2.6 cm），y 方向差了 1.43 m。这就是向量的好处 —— 你能看出“哪个方向错了”。

### 例 2：`RangeFactor`（1 维标量残差 + 流形，lesson1）

$$h(x) = \big\|T^{-1}p\big\| = \big\|\text{transformTo}(p)\big\|,\qquad e = h(x) - z$$

注意这里有两层“非线性”：先把世界系路标 $p$ 变到机器人自身坐标系（含旋转），再取模长。

**一个关键性质**：$\|T^{-1}p\|$ 只跟 $T$ 的平移有关，**跟旋转 θ 无关**（因为旋转不改变长度）。所以：

$$
\frac{\partial e}{\partial \theta} = 0
\quad\Longrightarrow\quad
\text{测距因子对 } \theta \text{ 一无所知}
$$

这就是 Lesson 1 第 3 节“θ 不可观”的代数解释：H 里 θ 那一列全是 0，信息矩阵在 θ 方向特征值为 0。

---

## 3. 雅可比矩阵到底有什么用？

### 3.1 它是“局部斜率”，把曲线拉直

残差 $e(x)$ 是非线性的（有 `sqrt`、有 `cos/sin`、有 `atan2`）。我们不会解非线性问题，所以做**一阶展开**：

$$
e(x \boxplus \delta) \;\approx\; e(x) + J\,\delta,
\qquad J = \frac{\partial e}{\partial x}
$$

$J$ 的形状是 **“残差维数 × 该变量的切空间维数”**。`OffsetFactor` 对 `p_from` 的雅可比是 $2\times2$：

$$
J_{p_\text{from}} = \frac{\partial (p_\text{to}-p_\text{from}-z)}{\partial p_\text{from}} = -I
=
\begin{pmatrix}-1&0\\0&-1\end{pmatrix}
$$

**怎么读这个矩阵**：第 1 行说的是“残差第 1 个分量（x 方向）对 $p_\text{from}$ 的 x、y 分别多敏感”。$-1$ 意味着“把 `p_from` 往 +x 挪 1 m，残差的 x 分量就减 1 m”。第 2 行同理。

> 所以雅可比就是**敏感度表**：某一行全 0 → 这个残差完全不关心某些变量（Lesson 1 的 θ 列）；某两列成比例 → 这两个变量分不开（秩亏）。

### 3.2 它是求解的“方向盘 + 油门”

把一阶展开代进目标函数：

$$
\|e + J\delta\|^2 = e^Te + 2e^TJ\delta + \delta^TJ^TJ\delta
$$

对 $\delta$ 求导并令其为 0：

$$
2J^Te + 2J^TJ\delta = 0
\;\;\Longrightarrow\;\;
\boxed{J^TJ\,\delta = -J^Te}
$$

- $J^T e$ 是**梯度**（往哪走能让代价下降）
- $J^TJ$ 是**信息矩阵 / 曲率**（这个方向有多“陡”，走多远合适）

**没有雅可比，就不知道往哪走。** 而且雅可比写错 ≠ 立刻报错 —— 它只是让“下山方向”指错，优化要么龟速、要么震荡、要么收敛到垃圾点。这就是为什么 Lesson 0/1 要在写完后**立刻**做数值自检：

```cpp
for (const auto& factor : graph) {           // lesson0 第 2 节
  lesson::printCheck(name, lesson::checkJacobian(*factor, probe));
}
// [ OK ] OffsetFactor(P0,P1)  max|J_ana-J_num| = 1.02e-09  rel = 5.10e-11
```

背后就是中心差分：

$$
J_{\text{num}}[:,k] \approx \frac{e(x + \epsilon e_k) - e(x - \epsilon e_k)}{2\epsilon}
$$

手写 $J$ 和它一比，相对误差 > 1e−5 就判 FAIL。Lesson 1 里故意写错的版本 `rel = 4.7e-01`，一眼就抓出来了。

### 3.3 它还顺手决定了“可观性”和“稀疏性”

| 你要问的问题 | 看雅可比/信息矩阵 |
| --- | --- |
| 这个变量能被约束住吗？ | $J$ 的对应列是否全 0（或近似线性相关） |
| 整张图有没有 gauge freedom？ | $\Lambda=J^TJ$ 有没有零特征值（上一问讲的秩亏） |
| 哪个位姿最不确定？ | $(J^TJ)^{-1}$ 的对角块 = 边缘协方差 |
| 为什么 iSAM2 快？ | $J$ 行只碰少数列 → 稀疏分解只需局部重算 |

**同一个矩阵，同时回答了“怎么解”和“解得好不好”。**

### 3.4 一个必须记住的约定：你返回的是“未白化”雅可比

GTSAM 的规矩是：**`evaluateError` 里的 $e$ 和 $H$ 都不乘噪声模型**，白化由 `NoiseModelFactor` 统一做（`WhitenSystem`：$A \leftarrow RA$, $b \leftarrow Rb$，$R=\Sigma^{-1/2}$）。

好处：**换噪声模型（高斯 → Cauchy → Diagonal）时，你的因子代码一行都不用改**。Lesson 2 就是用同一个 `RangeFactor` 跑三种噪声模型。

⚠️ 反过来，如果你在 `evaluateError` 里自己又除了 σ（“白化两次”），权重就变成 $1/\sigma^2$ 而不是 $1/\sigma$，结果会系统性偏。这是我上一轮修 lesson5 权重打印时踩的同一个坑（`Robust::weight` 要传**未白化**残差）。

### 3.5 流形上的雅可比（`Pose2` 要特别注意）

对向量变量，$\delta x$ 就是普通的几维向量。但 `Pose2=(x,y,θ)` 是**流形**，更新方式不是“加”而是 `retract`：

$$
T \boxplus \delta = T\cdot\text{Exp}(\delta),\qquad \delta\in\mathbb{R}^3\ (\text{切空间})
$$

所以 $J=\partial e/\partial \delta$ 是**对切空间**求导，维度固定是 3。你不需要自己推 $\partial\|\cdot\|/\partial\theta$ 的麻烦公式 —— `transformTo(p, &D_T, &D_p)` 会直接给你 $\partial l/\partial T$（2×3）和 $\partial l/\partial p$（2×2），你只需要链式法则：

$$
J_T = \underbrace{\frac{l^T}{\|l\|}}_{1\times 2}\cdot D_T\ (2\times3) = 1\times3,
\qquad
J_p = \frac{l^T}{\|l\|}\cdot D_p = 1\times2
$$

（这正是 `src/include/factors/range_factor.hpp` 里那 8 行代码。）

---

## 4. 完整流程图

```
你:   OffsetFactor(P0,P1,σ,z)      ← 只存 key + z + Σ
        │
        └→ graph.emplace_shared(...)  ← 图里多个指针，仍未算数

优化器（每轮迭代）:
        │
        ├─ ① graph.error(current)   → 逐因子: at(key)→evaluateError→whiten→½‖·‖²→求和
        │      （这一轮要不要接受？LM 用它比较）
        │
        ├─ ② graph.linearize(cur)   → 逐因子: evaluateError(+H)→whiten A,b→JacobianFactor
        │      └→ 摞成稀疏 A, b
        │
        ├─ ③ 解 (AᵀA)δ = Aᵀb        ← 这里用雅可比；秩亏就在这炸/被阻尼糊住
        │
        └─ ④ values = values ⊞ δ     ← 流形用 retract
                │
                └→ 步长够小？ 是→ 输出；否→ 回到①
```

---

## 5. 大二阶段最容易踩的 6 个坑

| 坑 | 症状 | 正确做法 |
| --- | --- | --- |
| 忘记 `using Base::evaluateError;` | 编译报“无匹配函数”/ 重载被隐藏 | 类里加这一行（`range_factor.hpp:44`） |
| 残差符号乱 | 优化“发散”或收敛到反方向 | 统一用 $e=h(x)-z$，别混 $z-h(x)$ |
| 自己又除了一次 σ | 权重变成 $1/\sigma^2$，结果系统性偏 | `evaluateError` 只返回**未白化**量 |
| H 的列数写成变量“存储维数” | 维度断言失败/结果错 | 列数 = **切空间**维数（`Pose2`=3，`Rot3`=3，`Point2`=2） |
| H 只在部分分支里填 | 自检偶发 FAIL，或 `error` 不变但 `δx` 乱飞 | 无论 `H` 是否为 `nullptr` 都走同一公式 |
| 分母 `‖l‖ → 0` | NaN | 加 `if (range > kEpsilon)` 保护（`range_factor.hpp:66`） |

---

## 6. 建议你现在动手做的 4 个实验（10 分钟）

都在 `src/lessons/lesson0_template.cpp` 里改，改完 `xmake run lesson0`：

1. **看残差怎么流动**：在 `OffsetFactor::evaluateError` 里加一行
   `std::cout << "p_from=" << p_from.transpose() << " p_to=" << p_to.transpose() << std::endl;`
   → 你会看到优化每迭代一次，它就被调用好几次（`error` 和 `linearize` 各一次）。
2. **感受 σ 的作用**：把 `kOffsetSigma` 从 0.05 改成 0.5，看第 1 节 `whiten(e)` 和 `error` 怎么变（残差不变，白化后小 10 倍，cost 小 100 倍）。
3. **感受雅可比写错的后果**：把 `*H_to = Matrix2::Identity();` 改成 `0.99 * Matrix2::Identity();`，第 2 节会从 `rel=1e-10` 变成 `rel≈1e-2` 的 FAIL；再跑第 3 节，LM 会多迭代很多次才收敛。
4. **看“不可观”长什么样**：第 5 节已经给你了 —— 零特征值那一列，就是 `H` 对这 2 个方向完全没感觉。

---

## 一句话复习

- **残差** = “预测 − 实测”，用来衡量“当前估计有多不合观测”。
- **噪声模型** = 把不同单位的残差换算成“相当于几个 σ”（白化），并且决定惩罚的力度。
- **雅可比** = 残差对变量的**局部敏感度表**；优化器靠它把非线性最小二乘**拉直**成线性方程 $J^TJ\delta=-J^Te$，同时它还能暴露不可观方向（零列/零特征值）。
- **一个因子最该写对的两件事**：`evaluateError` 的公式，以及它的雅可比 —— 后者一定要用 `lesson::checkJacobian` 做数值自检。

# lesson1中故意写错雅可比矩阵到底错在哪，pose2与point2又有什么区别

## 一、`BuggyRangeFactor` 到底错在哪

## 1. 先复习正确的推导

测距因子的完整计算是**两层嵌套**：

$$
\underbrace{l = T.\text{transformTo}(p) = R(-\theta)\,(p - t)}_{\text{①把世界系点变到机体系（2 维向量）}}
\qquad
\underbrace{e = \|l\| - z}_{\text{②取模长（标量）}}
$$

链式法则是：

$$
\frac{\partial e}{\partial \text{位姿}} =
\underbrace{\frac{\partial \|l\|}{\partial l}}_{\text{1×2 行向量}}
\cdot
\underbrace{\frac{\partial l}{\partial \text{位姿}}}_{\text{2×3，GTSAM 给的 } D_T}
= \underbrace{\frac{l^{T}}{\|l\|}}_{1\times2}\cdot D_T
$$

**这里的关键点是**：$\dfrac{\partial\|l\|}{\partial l} = \dfrac{l^T}{\|l\|}$ 里的 $l$ 必须是**机体系下的那个 $l$** —— 因为梯度的方向是在“你对谁求导时用的那套坐标”里定义的。$D_T$ 描述的是“机体系的 $l$ 怎么随位姿变”，所以乘在它左边的行向量也必须用机体系的 $l$。

## 2. 错误的那三行代码

```cpp
// src/lessons/lesson1_range_factor.cpp:61-71
// BUG：用"世界坐标系"下的方向 (p - t) 代替"机体系"下的方向 l
const Point2 l_world = p - T.translation();     // ← 只减了平移，没有旋转！
const double rw = l_world.norm();
J << l_world.x() / rw, l_world.y() / rw;        // ← 世界系方向
if (H_T) *H_T = J * D_T;                        // 拿世界系的 J 乘机体系的 D_T
if (H_p) *H_p = J * D_p;                        // 连带把对路标的雅可比也搞错了
```

它把 $l_{\text{world}} = p - t$ 当成了 $l$。但正确关系是 $l_{\text{world}} = R(\theta)\,l$ —— **差一个旋转矩阵 $R(\theta)$**。

于是：

$$
J_{\text{buggy}} = \frac{(R(\theta)l)^T}{\|l\|} = \frac{l^TR(-\theta)}{\|l\|}
= J_{\text{correct}}\cdot R(-\theta)
$$

也就是说，**错误的雅可比 = 正确的方向向量多转了 $\theta$ 角**。用 lesson1 的真实数字验证（truth $=(3,4,0.6)$，锚点 $(0,0)$）：

| 量 | 数值 |
| --- | --- |
| $l$（机体系） | $(-4.7346,\,-1.6074)$，$\|l\|=5$ |
| $J_{\text{correct}}$ | $(-0.9469,\;-0.3215)$ |
| $J_{\text{buggy}}$ | $(-0.6000,\;-0.8000)$ |
| 两者夹角 | **0.6 rad = 34.38°，恰好等于 $\theta$** |

## 3. 为什么这个 bug 特别“阴险”（三重伪装）

| 伪装 | 原因 | 后果 |
| --- | --- | --- |
| **维度全对** | $1\times2$ 乘 $2\times3$ $=1\times3$ ✱ 编译器只看形状 | 不会报错 |
| **残差完全正确** | 旋转保长 → $\|l_{\text{world}}\|=\|l\|$ → $e$ 一模一样 | `graph.error(values)` 一个数字都不差 → “cost 在下降”完全失效 |
| **θ=0 时恰好正确** | $R(0)=I$ | 如果测试点选在 θ=0 的平凡位形，永远发现不了 |

所以它只在 **`linearize()`（求解方向）** 那一步暴露。这也正是 Lesson 1 第 2 节必须专门做一次雅可比自检的原因：

```
[FAIL] BuggyRangeFactor(X0,L0)  max|J_ana-J_num| = 4.18e+01  rel = 4.73e-01
[FAIL] BuggyRangeFactor(X0,L1)  max|J_ana-J_num| = 2.68e+02  rel = 2.69e+00
[FAIL] BuggyRangeFactor(X0,L2)  max|J_ana-J_num| = 3.26e+02  rel = 3.28e+00
```

`rel = 0.47` 就是“差了 47%”，`rel = 2.69` 意味着误差比真值本身还大两倍多。**只有雅可比自检能抓到它** —— 因为 `error()` 根本看不见这个错误。

## 4. 后果：不是“收敛慢”，而是直接往山上走

lesson1 的初值是 $(1,-1,0.3)$，真值 $(3,4,0.6)$。手算初始代价（三条残差 $-3.5858,\ 0.9931,\ 4.3372$ m，$\sigma=0.01$ m）：

$$
\text{cost}_0 = \tfrac12\left[(358.6)^2+(99.3)^2+(433.7)^2\right] + \text{先验} \approx 1.63\times10^5
$$

走**一步** Gauss-Newton：

```
正确雅可比: graph.error = 43400.9        ← 1.63e5 → 4.34e4，下降 3.8 倍 ✓
错误雅可比: graph.error = 4.57547e+07    ← 1.63e5 → 4.58e7，上升 280 倍 ✗
```

为什么一步就能炸？因为测距因子的 $\sigma=0.01$，信息矩阵在平移方向非常“硬”（Lesson 1 第 5 节：权重是先验的 1e4 倍）。$J$ 方向偏 34°，GN 解出的步长 $\delta=-(J^TJ)^{-1}J^Te$ 就朝着错误方向猛冲 3~4 m，直接把位姿甩到锚点另一侧。

> 一个直观类比：你在山谷里，真正的最陡下降方向是“向北 34°”，但你按“向北”走。走一步不但没下坡，还爬上了对面的山脊。

## 5. 正确的写法（对比记忆）

```cpp
// src/include/factors/range_factor.hpp:57-73（正确版本）
const Point2 l = T.transformTo(p, H_T ? &D_T : nullptr, H_p ? &D_p : nullptr);
const double range = l.norm();

Vector1 e;
e << range - measured_;

Matrix12 J;                                  // ∂‖l‖/∂l，必须用 l（机体系）
if (range > kEpsilon) J << l.x() / range, l.y() / range;
else                  J.setZero();           // 路标正好在原点，方向导数不存在

if (H_T) *H_T = J * D_T;                     // 1×3
if (H_p) *H_p = J * D_p;                     // 1×2
```

**只有一处不同**：`l` 换成了 `l_world`。但这一处让梯度方向偏了整整 $\theta$。

## 6. 从这件事能学到的三条通用经验

1. **链式法则的中间量必须在同一个坐标系里**。$D_T$ 是“机体系 $l$ 的导数”，乘在它左边的必须是机体系的梯度。跨坐标系相乘时，先写清楚“这个量是在哪个系里定义的”。
2. **检验极限/对称性**：这个 bug 在 $\theta=0$ 时消失 → 说明**测试点必须选在非平凡位形**（`makeInitial` 用 $(1,-1,0.3)$、truth $\theta=0.6$，正是为此）。同理，如果 bug 在 $\theta\to 0$ 时消失，你就绝不能只用 θ=0 做回归测试。
3. **残差对 ≠ 雅可比对**。`error()` 只走“残差”那条路，完全绕开 $H$。写完因子必须 `lesson::checkJacobian`（中心差分），并且换 2~3 个不同的线性化点各测一次。

> 顺带一提：`H_p` 也一起错了 —— 因为它共用了同一个 $J$（$H_p = J\cdot D_p = l^TR(-2\theta)/\|l\|$，而正确是 $l^TR(-\theta)/\|l\|$）。**一个错误的中间量会污染所有下游雅可比**，这也是为什么自检要检查每一个 key 的 $H$。

---

## 二、`Pose2` 和 `Point2` 有什么区别

## 1. 一句话

- **`Point2` = 一个位置**，就是 $\mathbb{R}^2$ 里的向量。GTSAM 里它甚至是 `typedef Vector2 Point2` —— **就是一个 Eigen 定长向量，2 个 double**，没有旋转概念。
- **`Pose2` = 一个坐标系**（位置 + 朝向），是群 $SE(2)$ 的元素。头文件第一行就写着：

```cpp
class GTSAM_EXPORT Pose2 : public MatrixLieGroup<Pose2, 3, 3> {
  Rot2   r_;      // 朝向：存成 (cos, sin) 两个数
  Point2 t_;      // 位置：2 个数
  ...
};
```

注意这个 `<Pose2, 3, 3>`：**存储占了 4 个数（cos, sin, x, y），但“自由度 / 切空间维数”是 3**。这个“存储维数 ≠ 切空间维数”是流形编程里最重要的一个概念。

## 2. 对比表

| | `Point2` | `Pose2` |
| --- | --- | --- |
| 数学本质 | $\mathbb{R}^2$ 向量 | $SE(2)$ 李群（**非线性流形**） |
| 有几个自由度 | 2 | 3（$x,y,\theta$） |
| 内存里几个数 | 2 | 4（$R$ 存 $\cos,\sin$ + $t$ 存 $x,y$） |
| 能否直接相加 | 能：`p1 + p2` | **不能**：必须 `T * Exp(δ)` |
| 基本运算 | `+ - *`、`.norm()`、`.dot()` | `compose / between / inverse / transformTo / transformFrom / bearing / heading / retract / localCoordinates / Logmap / Adjoint` |
| 优化时的更新 | `x += δ` | `T = T * Pose2::Expmap(δ)`（`retract`） |
| 在因子里的 $H$ 列数 | **2** | **3** |
| 典型角色 | 路标、锚点、GPS 观测、`Pose2::translation()` 的返回值 | 机器人位姿、里程计、回环约束 |
| 两两“误差” | `(a - b).norm()` | `Pose2::Logmap(a.between(b))`（`lesson::poseError`） |

## 3. 为什么 `Pose2` 不能直接“加”—— 三个具体理由

**① 旋转不可交换**

```
先转 90° 再前进 1 m  →  终点完全不同  →  先前进 1 m 再转 90°
```
所以 `Pose2` 的“加法”必须是 $T \cdot \text{Exp}(\delta)$（先转再平移的顺序被群法则固定），而不是 $(x+\delta_x,\ y+\delta_y,\ \theta+\delta_\theta)$ 简单相加。

**② θ 有 ±π 回绕**

Lesson 3 第 1 节亲手演示了这一点：

```
θ 从 3.0 到 -3.0：
  朴素角度平均 = 0 rad（错，方向反了 180°）
  流形上取中点 = 3.14159 rad（≈ ±π，正确）
```
3.0 和 −3.0 其实只差 **0.28 rad**（跨过 ±π），但算术平均给出 0。任何“忘记 θ 是角度”的代码都会在这里出错。

**③ 平移和旋转耦合**

`between`、`localCoordinates`、`Logmap` 三个量定义都不一样，Lesson 3 的输出把它们并列出来：

```
A = (1, 2, 0.3),  B = (0.5, -0.2, 0.4)
A * B (compose)       = (1.53677, 1.95669, 0.7)
A.between(B)          = (-1.12781, -1.95398, 0.1)
A.localCoordinates(B) = -1.12781 -1.95398  0.1   ← 优化器实际用的 chart（平移直接相减）
Logmap(A.between(B))  = -1.22457 -1.89596  0.1   ← 精确 SE(2) 指数映射（V(θ)⁻¹ 修正）
```

`localCoordinates` 和 `Logmap` 在 θ→0 时一致，差值是二阶小量 —— **用哪个是“约定”，必须与优化器一致**。这也是为什么在流形上手写雅可比容易错：你写的是对**切空间**的导数，不是对“存储的 4 个数”的导数。

## 4. 它们在一个因子里怎么“见面”

`RangeFactor` 就一行把两者关系讲清了：

```cpp
const Point2 l = T.transformTo(p);   // Pose2 把 世界系的 Point2 变到 机体系
```

- **`Point2` 是“哪”，`Pose2` 是“在哪、朝哪”**。
- 只有 `Pose2` 有朝向，所以**只有它能定义“机体系”**（`transformTo` / `transformFrom` / `bearing` / `heading` 都是它的成员函数）。
- `Pose2::translation()` 返回 `Point2` —— 这就是二者的“桥”。但注意语义：`Pose2` 里的 $(x,y)$ 是“**这个坐标系的原点**在世界的位置”，`Point2` 的 $(x,y)$ 是“**一个点**”的位置。数值上一样，含义不同。

## 5. 一个能立刻验证的小实验（把两问串起来）

把 `lesson1` 的 `makeInitial` 里的初值 θ 从 `0.3` 改成 `0.0`：

```cpp
initial.insert(X(0), Pose2(1.0, -1.0, 0.0));   // 原来是 0.3
```

重新 `xmake run lesson1`，第 2 节会从

```
[FAIL] BuggyRangeFactor(X0,L0)  rel = 4.73e-01
```

变成

```
[ OK ] BuggyRangeFactor(X0,L0)  rel = 1.0e-10
```

因为 $\theta=0 \Rightarrow R(\theta)=I \Rightarrow l_{\text{world}} = l$，**错误公式退化成正确公式**。

这一个实验同时说明了两件事：
1. `Pose2` 的“错”只在 **θ 非零**时才显现 —— 因为 `Pose2 ≈ Point2 + 一个旋转`，旋转为零时二者等价；
2. 回归测试必须覆盖**非平凡位形**，否则 bug 会安稳地活在代码里。

---

## 一句话总结

- **错在哪**：把机体系的方向 $l$ 写成了世界系方向 $p-t$，等价于把梯度方向多转了 $\theta$（实测 34.38°）。残差没变、维度没错，所以只有雅可比自检能抓住；一步 GN 代价从 $1.6\times10^5$ 涨到 $4.6\times10^7$。
- **Pose2 vs Point2**：`Point2` 是 $\mathbb{R}^2$ 里的一个点（2 维、可相加）；`Pose2` 是 $SE(2)$ 里的一个坐标系（3 自由度、存 4 个数、必须用 `compose/between/retract` 操作）。因子里的 $H$ 列数分别是 2 和 3 —— 这个数字的差别，正是“流形”这个概念的全部来由。

# 边缘协方差与残差表分别表达什么意思，应该怎么读？

## 一句话对照

| 表 | 回答的问题 | 观察对象 | 单位 |
| --- | --- | --- | --- |
| **残差表** | 「这条**数据**跟当前估计有多不一致？」 | 因子（逐个） | 无量纲（σ 的倍数²）|
| **边缘协方差** | 「这个**状态量**本身还有多不确定？」 | 变量（逐个） | m²、rad² |

一个是**事后体检**（数据对不对得上），一个是**置信度报告**（状态定得牢不牢）。两张表都不是「真实误差」——真实误差要拿真值才能算，而真值在实际系统里没有。

---

## 一、边缘协方差怎么读

## 1.1 它是从哪来的

优化时我们解的是最小二乘，它的「曲率」就是**信息矩阵**（Hessian）：

$$
\Lambda = J^{T}\Sigma_{\text{meas}}^{-1}J,
\qquad
\Sigma_{\text{post}} = \Lambda^{-1}
$$

- $\Lambda$ 大 → 这个方向「很陡」→ 稍微偏一点代价就暴涨 → 状态被钉得很死。
- 求逆 → 得到后验协方差 $\Sigma_{\text{post}}$。

**「边缘」（marginal）的意思是**：把其它变量**积分掉**，只看某一个变量自己的不确定性。所以它**不是**「只考虑连着它的那些因子」，而是「整张图一起解完之后，它的残余不确定度」。

```cpp
Marginals marginals(graph, result);
Matrix3 cov = marginals.marginalCovariance(X(0));   // 3x3（Pose2 的切空间）
```

## 1.2 读一个 3×3 块（用 lesson1 §3b 的真实输出）

```
  边缘协方差 Cov(X0) [vx, vy, omega]:
0.000161147  3.35074e-05  -2.33579e-17
3.35074e-05  0.000125564   5.40308e-17
-2.33579e-17 5.40308e-17   0.25
```

**(a) 对角线 = 方差 → 开方得到 1σ**

| 量 | 方差 | σ（开方） | 含义 |
| --- | --- | --- | --- |
| vx | 1.611e-4 | **0.0127 m** | x 方向约 ±1.3 cm（68% 置信） |
| vy | 1.256e-4 | **0.0112 m** | y 方向约 ±1.1 cm |
| ω | 0.25 | **0.5 rad** | θ 完全没定住（±28.6°） |

**(b) 非对角 = 相关性**，要归一化成相关系数才可读：

$$
\rho_{xy} = \frac{\Sigma_{xy}}{\sigma_x\sigma_y} = \frac{3.35\times10^{-5}}{0.0127\times0.0112} \approx 0.236
$$

→ x、y 有**弱正相关**（x 估小了，y 也倾向估小一点），这意味着「x 单独偏 1.3 cm」这种说法不完全准确，误差是一块斜着的椭圆。

**(c) ω 那一行/列几乎全是 0，对角是 0.25** —— 这个信息量很大：

- 与 x,y 的相关性 ≈ 0：θ 的误差和位置误差**解耦**；
- 0.25 **正好等于先验的 $\sigma^2=0.5^2$**：说明 θ 的不确定性 100% 来自先验，测距因子贡献了 **0**。

这就是 lesson1 §3 用 Hessian 零特征值说明的同一件事（秩亏/不可观），只是这里从「方差」的角度再看一遍。

## 1.3 读一张表（lesson0 §4）

```
   点      sigma_x      sigma_y     sqrt(det)   信息源
   P0      0.0200       0.0200       0.0004   先验 + 2 条边
   P1      0.0455       0.0455       0.0021   2 条边
   P2      0.0455       0.0455       0.0021   2 条边（含冗余边）
```

- `sigma_x/sigma_y` 就是 `sqrt(cov(0,0))`、`sqrt(cov(1,1))`，**直接当 ±1σ 读**（单位 m）。
- `sqrt(det)` 是**不确定椭圆面积**的度量：2×2 时 $\sqrt{\det\Sigma} = \sigma_1\sigma_2$(若是各向同性)。它只适合**横向排序**（谁比谁更不确定），不能当物理量念出来。
- **P0 的 0.0200 就是先验的 σ**：说明位置几乎完全由先验钉住（两条边对它只是补充）。
- **P1/P2 = 0.0455 比单条边的 σ=0.05 小一点，但没到 σ/√2=0.035**：因为另一端的 P2 自己也不确定，误差可以在两点之间「让来让去」。**边缘方差永远取决于整张图的拓扑，不是单条边的 σ 之和**。

## 1.4 四个常见误读（重点）

| 误读 | 真相 | 你仓库里的反例 |
| --- | --- | --- |
| **σ 小 ⇒ 估计准** | 不对。它是**模型自认为**的置信度。σ 填小、漏因子、把外点当内点，都会得到「自信地错」 | lesson4 的 X15 是 GPS 跳变帧：普通最小二乘会把它压得很自信却错 2.6 m；加鲁棒核后 σ 反而变大，**更诚实** |
| **只看对角线** | 不对。强相关时「σ_x=0.1」不代表 x 独立地偏 0.1；要看整个块或椭圆 | lesson1 §3b 的 ρ≈0.236 |
| **协方差是全局有效** | 不对。它是**当前线性化点附近的一阶近似**；初值很差、强非线性时，它不反映真实的全局不确定性 | lesson3 大回环、lesson1 从 (1,−1) 出发时都强烈非线性 |
| **边缘 = 条件** | 边缘是「其它变量都被积分掉」；条件协方差（固定别的变量）会**更小**，不能混用 | 想固定别的量看相关性要用 `jointMarginalCovariance` |

> 进阶：想知道「最坏方向」，对协方差做特征分解即可：
> ```cpp
> Eigen::SelfAdjointEigenSolver<Matrix3> es(cov);
> // 半轴长度 = es.eigenvalues().cwiseSqrt()
> // 半轴方向 = es.eigenvectors()
> ```
> 最小特征值对应的方向，就是这张图**最不确定**的方向（lesson0 §5 那个「零特征值」的连续版本）。

---

## 二、残差表怎么读

## 2.1 每行是什么

代码在 `src/include/lesson_utils.hpp:107`：

```cpp
const double e = f->error(values);     // 单个因子的代价
total += e;                            // 加起来就是 graph.error(values)
```

而 `factor.error(x)` 的定义是（`NonlinearFactor.h:292`）：

$$
\text{error}_i = \tfrac12\left\|\Sigma_i^{-1/2}\big(h_i(x)-z_i\big)\right\|^2
= \tfrac12\sum_k \left(\frac{e_{i,k}}{\sigma_{i,k}}\right)^2
$$

**已经白化过了**，所以是无量纲的。三个字段：

| 字段 | 含义 | 用途 |
| --- | --- | --- |
| `# 序号` | 因子在图里的下标（`graph[i]`） | 定位/删除它；GNC 的 `knownInliers/knownOutliers` 就是这组下标 |
| `keys=[...]` | 它连了哪些变量 | 看它属于哪个传感器/哪一段 |
| `error=` | 白化残差平方和 | 一致性检查 |

## 2.2 逐个读（lesson1 §5 的真实输出）

```
--- factor errors ---
  # 0  keys=[x0]        error=12.495737   ← 位姿先验（中心在原点，σ=(1,1,0.5)）
  # 1  keys=[l0]        error=0.000909    ← 锚点先验（把路标钉死）
  # 2  keys=[x0, l0]    error=0.000909    ← 测距因子（σ=0.01 m）
  ...
  total = 12.497868
```

**换算口诀：σ 数 = √(2·error)**

| error | √(2·error) | 解读 |
| --- | --- | --- |
| 0.0009 | 0.043 σ | 几乎完美吻合（对应残差 0.43 mm，σ=1 cm） |
| 0.5 | 1 σ | 正常量级 |
| 12.4957 | **5.00 σ** | 严重不一致 |

第 0 行可以手算验证：估计值 $(3,4,0)$、先验 $(0,0,0)$、σ 为 $(1,1,0.5)$：

$$
\text{error} = \tfrac12\left[\left(\tfrac{3}{1}\right)^2+\left(\tfrac{4}{1}\right)^2+\left(\tfrac{0}{0.5}\right)^2\right] = \tfrac12(9+16)=12.5
$$

完全对上 → **先验「抱怨」了 5σ**。

## 2.3 致命陷阱：残差大 ≠ 影响大

先验 `error=12.5`，测距 `error=0.0009`，差 **1.4 万倍**；但测距的 σ 小 100 倍，**权重（1/σ²）大 1e4 倍**，所以 x,y 仍被测距拉到真值附近，先验只是「强烈抗议但没人听」。

**判据**：看「误差大 **且** 权重不小」的因子。等价做法是直接问因子要权重：

```cpp
double w = factor->weight(values);        // NoiseModelFactor::weight（NonlinearFactor.h:282）
```

这就是为什么 lesson5 不打印 cost 而打印 **Cauchy 权重 / GNC 权重** —— 权重才是「谁有话语权」的直接答案。

## 2.4 鲁棒核会让残差表「骗人」

`Robust` 模型的 `error` 用的是**截断损失**（$0.5\rho(d^2)$），外点的 error 会被**封顶**：

- 一个偏 40σ 的外点，error 可能只有 2~3（不是 800）；
- 表中看不出它是外点！

所以用鲁棒核时，**残差表必须配权重表一起看**：

```
lesson5 §2：
  X1->X10 内点  Cauchy weight=0.9996   GNC=1.0000
  X0->X11 外点  Cauchy weight=0.1656   GNC=0.0000   ← 这才是判决
```

## 2.5 四个常见误读

| 误读 | 真相 |
| --- | --- |
| 跨噪声模型比较 cost | **无意义**。所以 lesson4 标了 `cost(该模型下)`，lesson5 干脆不打印 |
| error 小 ⇒ 因子不重要 | 反过来才对：它被完美满足，说明它「很满意」，不代表它没出力 |
| error 大 ⇒ 因子是错的 | 也可能只是初值差；**必须看优化后**的值 |
| total 越接近 0 越好 | 不一定。total 太小可能是**σ 填太大**（权重太小）或过拟合；合理量级 ≈ 残差维度数量级（chi² 期望） |

---

## 三、两张表怎么配合着用（实战）

## 3.1 一张 2×2 判据表

| | **协方差小** | **协方差大** |
| --- | --- | --- |
| **残差大** | ⚠️ **最危险**：模型很自信，但数据和估计对不上 → 外点、σ 填错、模型错、秩亏被先验掩盖 | 正常：数据不吻合 + 本身也不确定 → 需要更多信息 |
| **残差小** | ✅ 理想：既吻合又钉得牢 | 🤔 可疑：因子被满足了但没提供信息 → 权重被鲁棒核压低，或雅可比在该方向≈0（不可观） |

## 3.2 用你仓库里的三个例子验证这个判据

**(a) lesson1 §3b → 落在「残差小 + 方差大」格**

```
# 2  keys=[x0,l0]  error=0.000909      ← 测距残差 0.04σ，几乎完美
omega 方差 = 0.25（= 先验 σ²）          ← θ 却完全没定住
```
→ 结论：测距因子「满意但不给 θ 提供信息」。这正是 Lesson 1 的教学重点。

**(b) lesson4，普通高斯 GPS → 落在「残差大 + 协方差小」危险格**

```
外点帧平均位置误差 = 2.658 m，X15 sigma = 0.14（很小）
```
→ 模型自信地把跳变帧当真，位置却错 2.6 m。**这就是「自信地错」**。
换成 Cauchy 后：外点帧误差 0.166 m，X15 的 σ 反而升到 0.31 —— 变成诚实的「不确定」。

**(c) lesson0 §4 → 用来看「谁的信息多」**

```
P0 σ=0.0200（先验 + 2 条边）
P1/P2 σ=0.0455（只靠边）
```
残差表此时几乎全是 0（优化收敛），**信息全在协方差里**：这张表告诉你「如果再加一个传感器，加在 P1/P2 上收益最大」。

## 3.3 推荐的使用顺序

1. **优化前**：跑残差表 → 找离群观测 / 判断初值是否离谱；
2. **优化后**：再跑残差表 → 有没有「死活降不下去」的因子（外点、模型错、回环冲突）；
3. 看**权重**（或鲁棒核权重）→ 谁被降权、谁在主导；
4. 看**协方差** → 哪些状态没定住、哪个方向最不确定（特征向量）；
5. 交叉对照 2×2 表 → 判断是「数据问题」还是「信息不足」；
6. 只有**有真值时**才能算真实误差（lesson 里打印的 `误差` 列）—— 这是校准 σ 的唯一手段。

---

## 四、记住这两句话

- **残差表问的是「模型自洽吗」**：它衡量的是 `数据 ↔ 估计` 的差，且已经被 σ 归一化；**必须配合权重读**，否则会被「大声但没分量」的因子骗。
- **边缘协方差问的是「模型有多确定」**：它是 `信息矩阵的逆` 的对角块，**开方才是 σ**；它反映的是**有效信息**（鲁棒核降权后会变大），但**不保证准**——σ 填错时它会自信地骗你。

> 一句话：**残差表看「有没有人吵架」，协方差看「话事人说话算不算数」，两张表一起看才知道是「数据脏」还是「信息少」。**

# Gaussian，Cauchy，Huber三种噪声模型有什么异同？是什么导致了它们优化出来结果不同？分别适合用于什么环境？

我把 GTSAM 里三个核的**原始公式**（`gtsam/linear/LossFunctions.h`）和你 lesson2 的**实测数字**放在一起讲，这样"为什么结果不同"能直接对上。

## 一、同：它们都是 M-estimator，走同一套机制

三者共用同一个抽象：

$$
\min_x \sum_i \rho\!\left(d_i(x)\right),
\qquad d_i = \sqrt{e_i^{T}\Sigma_i^{-1}e_i}
\;\;(\text{白化后的马氏距离，单位是 }\sigma)
$$

- **只改 $\rho$，不改残差公式**。你的 `evaluateError` 一行都不用动（lesson2 里同一个 `RangeFactor` 跑了三种模型）。
- 都用 **IRLS**（迭代重加权最小二乘）实现：每轮按当前残差算权重 $w(d)=\psi(d)/d$（$\psi=\rho'$），解一个**加权**最小二乘 $J^TWJ\,\delta=-J^TW e$。
- 都在**白化之后**起作用，所以参数 $k$（或 $c$）的单位是「**几个 $\sigma$**」，与传感器量纲无关。
- 在 GTSAM 里都包在同一层：`noiseModel::Robust::Create(mEstimator, gaussian)`，内层高斯负责白化和提供基础协方差。

## 二、异：$\rho$ 的形状决定了「一个外点能造成多大拉力」

| | **Gaussian (L2)** | **Huber** | **Cauchy** |
| --- | --- | --- | --- |
| 损失 $\rho(d)$ | $\tfrac12 d^2$ | $\tfrac12 d^2\ (d\le k)$；$\tfrac12k^2+k(d-k)\ (d>k)$ | $\tfrac12k^2\log\!\left(1+\tfrac{d^2}{k^2}\right)$ |
| 影响函数 $\psi=\rho'$ | $d$（**无界**） | $d$ 或 $k$（**饱和**） | $\dfrac{k^2d}{d^2+k^2}$（先升后**回落**） |
| 权重 $w=\psi/d$ | $1$ | $1$ 或 $k/d$ | $\dfrac{k^2}{d^2+k^2}$ |
| 凸性 | **严格凸**（唯一解） | **凸**（唯一解，C¹ 不 C²） | $d>k$ 时 $\rho''<0$ → **非凸** |
| 大外点的代价（d=25σ） | $312.5$ | $32.7$ | $3.2$ |
| GTSAM 默认 $k$ | — | 1.345（≈95% 高斯效率） | **0.1**（很激进，务必显式传参） |

**权重随残差衰减的速度**（这是最实用的直觉，d 一律以 σ 为单位）：

| $d$ | 1σ | 2σ | 3σ | 5σ | 10σ | 25σ |
| --- | --- | --- | --- | --- | --- | --- |
| Gaussian | 1 | 1 | 1 | 1 | 1 | 1 |
| Huber (k=1.345) | 1 | 0.67 | 0.45 | 0.27 | 0.13 | 0.054 |
| Cauchy (k=1) | 0.50 | 0.20 | 0.10 | 0.038 | 0.010 | **0.0016** |

```
w(d)
1.0 ┤ Gaussian ────────────────
    │ Huber  ─────╮
0.5 ┤            ╰──────╮            ← 按 1/d 衰减，"永远还在拉"
    │ Cauchy ╮           ╰──────     ← 按 1/d² 衰减，"几乎放手"
0.1 ┤        ╰──╮
    └────┬────┬───┬────┬────┬──► d (σ)
         1    2   3    5   10
```

三个关键区别一句话：

- **Gaussian**：外点残差 2 倍 → 拉力 2 倍（平方代价）→ **一个 25σ 的外点等价于 625 个 1σ 内点**，直接把解拖走。崩溃点 = 0。
- **Huber**：$d>k$ 后拉力**恒为 $k$**（不再增长）→ 外点最多相当于一个"边界内点"。**凸**，结果与初值无关，可复现。
- **Cauchy**：$d\gg k$ 时拉力**趋近 0**（redescending）→ 真正"无视"外点；但代价**非凸**，会被初值左右。

## 三、为什么会优化出不同结果：三条独立机制

以 lesson2 同一份数据（真值 (2,1)，7 个锚点，其中 **2 个外点 +2.5 m**，σ=0.1，即 **25σ**）为例：

```
Gaussian(sigma=0.1)  est=( 1.036, 0.469)  xy_err=1.1004   cost=387.4
Cauchy(k=1.0)        est=(-0.440,-0.181)  xy_err=2.7111   cost=11.9
Huber(k=1.345)       est=( 1.919, 0.959)  xy_err=0.0909   cost=64.4
Cauchy（用 Huber 解做初值）xy_err=0.003
```

**机制 1：目标函数本身就不同 → 极小点不同**

L2 的目标里，两个 25σ 外点贡献 $2\times312.5=625$；而把解挪到使它们残差变小、代价是让 5 个内点各偏 ~0.7σ（约 $5\times0.25$）。**数学上最优解确实会偏**——所以 Gaussian 的 1.10 m 不是"优化没收敛"，而是「L2 意义下的正确答案」。Huber/Cauchy 把外点代价压到 32.7 / 3.2，内点于是重新占主导 → 0.09 m。

**机制 2：权重 = 有效噪声 → 谁被听见**

IRLS 里权重等价于把该因子的 $\sigma^2$ 放大 $1/w$ 倍。Cauchy 在 25σ 处 $w=0.0016$，等于把 σ 从 0.1 放大到 2.5 m，**这个因子事实上退出了优化**。所以最终解是"内点说了算"。

**机制 3：凸性 → 你只能找到「某个」极小**

这是 lesson2 最有价值的一段：**同一个 Cauchy，从 (0,0) 出发得 2.71 m，从 Huber 的解出发得 0.003 m。**

```
Cauchy（从 (0,0) 出发）的权重：
  z[0] |e|=1.760  weight=0.003     ← 内点被当成外点压掉了
  z[1] |e|=0.140  weight=0.336     ← 外点反而有 0.34 的权重
  z[2] |e|=0.913  weight=0.012
  z[3] |e|=0.035  weight=0.891     ← 外点拿到 0.89
  ...
```

因为 Cauchy 是 **redescending（权重会回落到 0）**，它既能"无视坏点"，也能"无视好点"——只要后者残差暂时看起来大。非凸 ⇒ 存在多个局部极小，落进哪个由初值决定。Huber 的权重是**单调不增**的，代价是**凸**的，所以永远只有一个谷底，与初值无关。

> 这也是 lesson5 里 **GNC** 的由来：GNC 不是另一个核，而是**把非凸核"退火"成一串凸问题**（μ 从大逐步收紧），用来"把 Cauchy/TLS 的全局最优找出来"。

## 四、分别适合什么环境

| | **Gaussian** | **Huber** | **Cauchy** |
| --- | --- | --- | --- |
| 内点比例 | ≈100% | 90~99% | 70~90%（更好初值可更低） |
| 外点幅度 | 没有，或前端已剔除 | 中等（3~10σ） | 很大（>10σ，甚至完全错误） |
| 是否需要凸/唯一解 | 需要 | **需要** | 不要求（但要有好初值/多初值/GNC） |
| 初值质量要求 | 低 | 中 | **高** |
| 典型场景 | 标定良好的里程计、IMU 预积分、激光 ICP 正常帧、**任何需要正确协方差的地方**、边缘化/iSAM2 | **在线融合的默认选择**：VIO 里程计、GPS 融合、轮速计打滑、雷达里程计偶发退化 | 回环检测、视觉特征匹配、UWB/测距多径、雷达多目标、任何"外点可能离谱"的前端 |
| 代价 | 外点一多就崩 | 大外点仍留偏差（$k/d$ 衰减慢） | 非凸；可能误伤内点；结果不可复现 |

**选择建议（工程顺序）**：

1. **先问"有没有外点"**。没有 → 用 Gaussian，因为你需要它**凸、唯一、协方差语义正确**。
2. **不确定 / 要上线跑** → **Huber 是安全默认**。凸性保证结果可复现，对 3~5σ 的污染已足够；经典 $k=1.345$ 在纯高斯下还能保留 ~95% 效率。
3. **外点可能离谱，且你有不错的初值**（例如回环前已用 PCM/RANSAC 粗筛）→ **Cauchy**，$k$ 显式给（1.0~2.38 常见），并且**最好配合 GNC 或"大 $k$ 起步、逐步收紧"**。
4. **外点占多数且彼此自洽** → 换核没用（lesson5 §5 实测：GNC 反而把 3 个真回环全丢掉）。这时必须靠**前端一致性检验（PCM/成对一致性）** 先降外点比例。

## 五、$k$ 怎么选（以及两个坑）

- $k$ 的物理含义：**"从几个 σ 开始不再完全相信"**。取内点白化残差分布的 95~99% 分位即可。可以先跑一次 Gaussian，打印 `sqrt(2*error)` 的直方图，看尾部从哪里开始变长。
- **$k$ 太小 → 误伤内点**（lesson2 里 Cauchy $k=1$ 在坏初值下把 5 个内点压到 0.003）；**$k$ 太大 → 退化成 L2**（$k\to\infty$ 时 $w\to1$）。
- ⚠️ **GTSAM 的 `Cauchy` 默认 $k=0.1$**，在 σ 单位为 1 的尺度上等于"超过 0.1σ 就不信"，非常激进。`Huber` 默认 1.345 是合理的。**永远显式传 $k$**。
- ⚠️ 顺序不能反：核作用在**白化**后的距离上。如果 σ 填错 10 倍，$k$ 的含义也跟着错 10 倍。

## 六、三个常见误区

| 误区 | 事实 | 你仓库里的证据 |
| --- | --- | --- |
| "换了鲁棒核就不用管初值" | Cauchy/Tukey 类是非凸的，初值差会掉进伪解 | lesson2：Cauchy 从 (0,0) 出发 → 2.71 m；换初值 → 0.003 m |
| "cost 可以跨核比较" | 三种 $\rho$ 的定义不同（L2 的 $0.5d^2$ vs Cauchy 的有界对数），数值不可比 | lesson4 标了 `cost(该模型下)`；lesson5 干脆不打印 cost |
| "鲁棒核只影响解，不影响协方差" | 权重就是有效信息：$w<1$ 等价于 $\sigma^2$ 放大 $1/w$ → **协方差会变大** | lesson4：X15 是跳变帧，普通融合 σ=0.14（自信地错），鲁棒后 σ=0.31（更诚实） |
| "外点多也不怕，核能兜住" | 影响函数有界/回落 ≠ 崩溃点无限；多数派自洽外点会让 GNC/核一起失效 | lesson5 §5：4 个自洽外点把 3 个真回环全判成外点 |

## 七、一句话总结

> **三者只差一个 $\rho$，而这个 $\rho$ 决定了「一个外点最多能拉多远」**：
> Gaussian 的拉力无界（凸、唯一、但会被拖走），Huber 的拉力封顶（凸、可复现、但大外点仍留偏差），Cauchy 的拉力会回落到 0（能真正无视离谱外点，代价是非凸、吃初值）。
> **没有"最好的核"，只有"匹配噪声统计的核"**：干净高斯的场景用 Gaussian，要上线要确定性用 Huber，前端脏但初值可用时用 Cauchy（并配 GNC）。

# gtsam的key是干什么用的，因子如何知道自己要取用什么数据

刚跑了个 5 行程序确认编码方式，答案可以给得很具体。

---

## 一、Key 是什么：一个 64 位整数，充当「变量的地址」

```cpp
typedef std::uint64_t Key;        // gtsam/base/types.h:43
```

`Values` 就是一张 **key → 值** 的表（`Values.h:74` 的 `KeyValueMap values_`），所以 **Key 的本质是「变量在 `Values` 里的主键/门牌号」**。没有 key，你无法从一张类型擦除的表里找回 `Pose2`、`Point2`、`double` 这些值。

## 1.1 两种写法，同一个东西

```cpp
// 写法 A：裸整数（GTSAM 官方例子常见）
values.insert(0, Pose2(0,0,0));
graph.emplace_shared<BetweenFactor<Pose2>>(0, 1, odom, noise);

// 写法 B：Symbol（带命名空间，教程里用这个）
values.insert(X(0), Pose2(0,0,0));                 // X = symbol_shorthand::X
graph.emplace_shared<BetweenFactor<Pose2>>(X(0), X(1), odom, noise);
```

`X(j)` 就是 `Symbol('x', j)`（`Symbol.h:171`），一行包装。

## 1.2 Symbol 的位布局（实测，不是猜的）

```
X(0) = 0x7800000000000000      // 'x' = 0x78 在高 8 位，下标 0 在低 56 位
L(3) = 0x6c00000000000003      // 'l' = 0x6c 在高 8 位，下标 3 在低位
L(3) & 0xFF      = 0x03        // ← 取下标
L(3) >> 56       = 0x6c        // ← 取字符
```

所以：**高 8 位 = 字符，低 56 位 = 下标**。GTSAM 还提供了两个便捷函数：

```cpp
symbolChr(key)    // → 'l'
symbolIndex(key)  // → 3
```

> lesson1 里打印因子名字用的就是这套 API：
> ```cpp
> lesson::printCheck("RangeFactor(X0,L" +
>                        std::to_string(symbolIndex(factor->key<2>())) + ")",
>                    lesson::checkJacobian(*factor, initial));
> ```
> `key<2>()` 是第二个 key（即 `L(i)`），`symbolIndex(...)` 取出下标 → 打印出 `L0/L1/L2`。
> 早期版本写的是 `factor->key<2>() & 0xFF`（靠位布局硬取），行为一样但语义不自解释，已改掉。

## 1.3 命名空间隔离 —— 这才是 Symbol 存在的理由

如果全用裸整数，`X(0)` 和"第 0 个路标"就撞号了。用字符分区：

| 字符 | 惯例含义 | 教程里的用法 |
| --- | --- | --- |
| `x` | 位姿 Pose2/Pose3 | `X(i)` |
| `l` | 路标 / 锚点 | `L(i)` |
| `v` | 速度 | (imu.hpp) |
| `b` | IMU 偏置 | (imu.hpp) |
| `p` | 点 | lesson0 的 `P(i)` |

多机器人还有 `LabeledSymbol('x', robotId, index)`（配 `MultiRobotKeyFormatter` 打印），避免两个机器人的 `x0` 撞车。

---

## 二、Key 的 7 项作用（不只是「取数」）

| # | 作用 | 在哪体现 |
| --- | --- | --- |
| 1 | **从 `Values` 里取数** | `values.at<Pose2>(key)` |
| 2 | 定义**因子的连接关系** | `factor->keys()`；因子图 = 变量节点 + 因子节点 |
| 3 | 决定线性化后 **Jacobian 的列块位置**（Scatter） | `JacobianFactor(keys_, dims_, ...)` |
| 4 | 决定**消元顺序 / 填充 / iSAM2 增量范围** | `Ordering::Colamd(graph)`、Bayes tree 的子树 |
| 5 | **查询边缘协方差** | `marginals.marginalCovariance(X(0))` |
| 6 | **类型命名空间隔离** | 见上表；避免"位姿和路标撞号" |
| 7 | 打印 / 可视化 / 序列化的稳定标识 | `graph.print()`、`saveGraph("g.dot")`、`gtsam` 文件 |

第 3~5 点很关键：**key 的排列顺序就是矩阵的排列顺序**，所以 key 不只是"名字"，它还决定了稀疏结构和求解代价（Lesson 3 §2 打印的邻接表就是按 key 排的）。

---

## 三、因子怎么知道「去取什么数据」：三层信息拼起来的

关键结论先说：**因子并不是"知道数据在哪"，它只知道「第 i 个参数去 `keys_[i]` 取，并按模板里第 i 个类型还原」。**

## 3.1 第一层：编译期 —— 模板参数给出「类型清单」

```cpp
class RangeFactor
    : public gtsam::NoiseModelFactorT<gtsam::Vector, gtsam::Pose2, gtsam::Point2> {
//                                  ↑残差类型    ↑第1个参数类型  ↑第2个参数类型
```

类型是**写死在类型系统里**的，运行时不需要 RTTI 探测（对比：`Values` 是类型擦除的，恢复类型靠 `at<T>`）。

## 3.2 第二层：构造期 —— `keys_` 给出「取件清单 + 顺序」

```cpp
RangeFactor(Key poseKey, Key pointKey, double measured, const SharedNoiseModel& noiseModel)
    : Base(noiseModel, poseKey, pointKey), measured_(measured) {}
//      ↑ 基类把 (poseKey, pointKey) 存进 keys_，顺序 = 你传的顺序
```

## 3.3 第三层：运行期 —— 把清单展开成函数调用（这是全部"魔法"）

`gtsam/nonlinear/NoiseModelFactorN.h:424-429`（真实源码）：

```cpp
template <std::size_t... Indices>
inline Vector unwhitenedError(gtsam::index_sequence<Indices...>,
                              const Values& x, OptionalMatrixVecType H) const {
  if (H) {
    return evaluateError(x.at<ValueTypes>(keys_[Indices])...,      // ← 取数
                         (*H)[Indices]...);                        // ← 回填雅可比
  } else {
    return evaluateError(x.at<ValueTypes>(keys_[Indices])...);
  }
}
```

对 `RangeFactor(X(0), L(0), ...)` 展开后等价于：

```cpp
evaluateError( values.at<Pose2>(keys_[0]),      // = values.at<Pose2>(X(0))
               values.at<Point2>(keys_[1]),     // = values.at<Point2>(L(0))
               &H[0], &H[1] );                  // H 也按同样顺序返回
```

**`Pose2` 从哪来？** 来自模板参数，不是你传的。
**`X(0)` 从哪来？** 来自 `keys_`，是你构造时传的。
**`values` 从哪来？** 优化器/调用者当参数传进来的。

## 3.4 `Values` 那侧的查找（`Values-inl.h:259-271`）

```cpp
KeyValueMap::const_iterator item = values_.find(j);
if (item == values_.end()) throw ValuesKeyDoesNotExist("at", j);   // ← 键不存在
auto h = internal::handle<ValueType>();
return h(j, item->second.get());                                   // ← 类型不符 → ValuesIncorrectType
```

## 3.5 整条链路

```
你的代码                                   GTSAM 内部
─────────────────────────────────────────────────────────────────
initial.insert(X(0), Pose2(...))        → values_[0x7800…00] = GenericValue<Pose2>
graph.emplace_shared<RangeFactor>(      → factor->keys_ = {0x7800…00, 0x6c00…00}
        X(0), L(0), z, sigma)              factor->measured_ = z
                                           factor->noiseModel_ = Σ          ← 构造完，没算任何数

graph.linearize(values)                 → 对每个因子：
   │                                       keys_  →  列块位置（Scatter）
   └─ factor->unwhitenedError(values,&H) → values.at<Pose2>(keys_[0])
                                           values.at<Point2>(keys_[1])
                                           → 你的 evaluateError(...)  ← 唯一你写的公式
                                           → whiten(H, e)  →  A δx = b
```

**一句话**：Key 是「取件单上的编号」，模板参数是「取件时要验的证件类型」，`Values` 是「仓库」，`evaluateError` 是「你收到货之后干的活」。

---

## 四、key 用错的三种后果（按危险程度排序）

| 错误 | 后果 | 是否好排查 |
| --- | --- | --- |
| **key 不在 `Values` 里** | 抛 `ValuesKeyDoesNotExist: Attempting to at the key "p0", which does not exist` | ✅ 立刻炸 |
| **key 在，但类型不符** | Debug 下断言 `Values::atRef: incorrect ValueType`；**Release 下 GTSAM 用 `static_cast`，可能直接读到垃圾内存** | ⚠️ Release 下很危险 |
| **类型相同但 key 顺序颠倒** | **完全不报错，模型静默变成另一个意思** | ❌ 最难发现 |

第 3 种举例（lesson0 的 `OffsetFactor`）：

```cpp
// 正确：z ≈ p_to - p_from
OffsetFactor(P(i), P(j), z, noise);
// 写反：残差变成 (p_i - p_j) - z，符号和方向全反 —— 编译照过，优化照样"收敛"
OffsetFactor(P(j), P(i), z, noise);
```

`RangeFactor` 之所以"幸运"，是因为 `Pose2`/`Point2` 类型不同，写反会编译失败：

```cpp
RangeFactor(L(0), X(0), ...);   // ✗ 编译错误：Point2 传给 const Pose2&
```

> **一个新手高频坑**：裸整数 `0` 和 `X(0)` **不是同一个 key**（`0` vs `0x7800…00`）。混用会造成"图里有个孤立节点 / key 找不到"。要么全用裸整数，要么全用 Symbol，别混。

## 4.1 一个真实案例（你仓库里踩过）

lesson0 早期版本里，自检用的是：

```cpp
Values probe;
probe.insert(P(i), ...);
probe.insert(P(j), ...);
lesson::checkJacobian(*factor, probe);   // 但 factor 的 keys 是 P(0), P(1)
```

当时因为"图的第 k 个因子"和"第 k 条边"索引错位，取到了另一条边的因子，它的 key 在 `probe` 里不存在 → 直接抛出

```
terminate called after throwing an instance of 'gtsam::ValuesKeyDoesNotExist'
  what():  Attempting to at the key "p0", which does not exist in the Values.
```

这正好说明：**因子取数只看 `Values`，不看图**。`Values` 不全，它一步都走不了。

---

## 五、常用 API 速查

```cpp
// 从 Values 取
values.at<Pose2>(X(0));                 // 不存在 → ValuesKeyDoesNotExist；类型错 → ValuesIncorrectType
values.exists(X(0));                    // 先问再取
values.erase(X(0));                     // 边缘化后删除

// 从因子问 key
factor->keys();                         // KeyVector
factor->key<1>(); factor->key<2>();     // 第 1/2 个（1-based）

// 拆解 Symbol
symbolChr(key); symbolIndex(key);

// 按字符批量过滤（例如只取所有位姿）
Values poses = values.filter(Symbol::ChrTest('x'));   // Symbol.h:120

// 图里所有 key（有序）
KeyVector ks = graph.keys();

// 打印时换 formatter
graph.print("graph: ", gtsam::MultiRobotKeyFormatter);
```

---

## 六、给变量起 key 的建议

1. **一类变量一个字符**：`x` 位姿、`l` 路标、`v` 速度、`b` 偏置、`k` 内参…… 这样 `graph.print()` 和 DOT 图一眼能看懂。
2. **下标连续、从 0 开始**：iSAM2 的增量更新、`Ordering`、可视化都假设 key 有规律；跳跃的 key 会让消元顺序变差（填充变多）。
3. **不要复用已删除的 key**：iSAM2 里 `marginalizeLeaves` 之后，那些 key 的线性化点被固定，复用它会产生"幽灵约束"。
4. **多机器人/多会话用 `LabeledSymbol`**：`LabeledSymbol('x', robotId, i)`，配 `MultiRobotKeyFormatter`。
5. **对外接口别暴露裸整数**：一旦发布过带裸整数的图/文件，之后改成 Symbol 就不兼容了。

---

## 七、一句话总结

> **Key = 变量的 64 位身份证（高 8 位是命名空间字符，低 56 位是编号）。**
> 因子在**编译期**由模板参数知道"第 i 个参数是什么类型"，在**构造期**由 `keys_` 记住"第 i 个参数是谁"，在**运行期**通过
> `evaluateError(values.at<Type_i>(keys_[i])...)`
> 把两者拼起来取数。所以因子永远不知道"数据存在哪"，它只负责"按清单去 `Values` 取件、按类型验货、算残差、给雅可比"。

---

## 八、相关源码位置（想深挖时按图索骥）

| 内容 | 文件:行 |
| --- | --- |
| `typedef std::uint64_t Key` | `gtsam/base/types.h:43` |
| `Symbol` 类、`symbolChr()` / `symbolIndex()`、`symbol_shorthand::X/P/L` | `gtsam/inference/Symbol.h` |
| `Symbol::ChrTest`（按字符过滤 `Values`） | `gtsam/inference/Symbol.h:120` |
| `Values::KeyValueMap values_` | `gtsam/nonlinear/Values.h:74-79` |
| `Values::at<T>` 的查找 + 类型检查 | `gtsam/nonlinear/Values-inl.h:259-271` |
| `ValuesKeyDoesNotExist` / `ValuesIncorrectType` | `gtsam/nonlinear/Values.h:440` 起 |
| `unwhitenedError` 展开 `keys_[Indices]` | `gtsam/nonlinear/NoiseModelFactorN.h:417-433` |

# lesson0TODO1的雅可比函数该怎么写，为什么？

## 分析过程（先说我查了什么）

我读了 `src/lessons/lesson0_template.cpp` 的当前版本（包括你未提交的改动）、`src/include/lesson_utils.hpp`、以及同类已成品因子 `src/include/factors/range_factor.hpp`（lesson1 的 1 维测距因子），然后在 `/tmp` 里写了 3 个独立小程序做了**实测**（没有改动仓库任何文件）：

| 验证项 | 结果 |
| --- | --- |
| `NoiseModelFactorT<float, Point2, Point2>` 能不能编译（你现在的写法） | ❌ `error: could not convert ... from 'float' to 'gtsam::Vector'`（`NoiseModelFactorN.h:364`） |
| `NoiseModelFactorT<gtsam::Vector1, ...>` + 1×2 雅可比 | ✅ `-fsyntax-only` 通过 |
| H_from/H_to 与中心差分对比（点 (0.3,-0.2)→(1.9,1.9)，z=1.7） | ✅ `max_abs_diff = 2.08e-10` |
| 距离三角形图（3 点 3 边）Hessian 零特征值个数 | ✅ **nullity = 3**，特征值 `-3.5e-14, -6.7e-15, 1.0e-13, 25.05, 1174.95, 1200` |
| 给 P0 加先验后 | ✅ **nullity = 1**（只剩绕 P0 的旋转） |
| 第 3 节那种「初值整体平移」在距离因子下的残差 | ⚠️ `error = 9.86e-30`（平移不改变距离 → 初值已经是最优解，LM 一步不走） |

下面是结论和原因。

---

## 1. 雅可比：一条**单位行向量**，1×2

设 $\Delta = p_{to} - p_{from}$，$r = \|\Delta\|$，$e = r - z$（标量）：

$$
\frac{\partial e}{\partial \Delta} = \frac{\Delta^{\mathsf T}}{\|\Delta\|} = \hat u^{\mathsf T}\ (1\times2),
\qquad
\frac{\partial \Delta}{\partial p_{to}} = +I,\quad
\frac{\partial \Delta}{\partial p_{from}} = -I
$$

$$
\boxed{H_{to} = +\hat u^{\mathsf T} = \frac{\Delta^{\mathsf T}}{\|\Delta\|},\qquad
H_{from} = -\hat u^{\mathsf T} = -\frac{\Delta^{\mathsf T}}{\|\Delta\|}}
$$

也就是说 TODO 提示里的「$(\Delta/\|\Delta\|)^{\mathsf T}$ 乘 ±I」，**±I 只贡献一个符号**——因为 $\partial\Delta/\partial p$ 就是 $\pm I$，而 $1\times2$ 乘 $2\times2$ 只把符号带出来，不会把行向量变成矩阵。

## 2. 代码怎么写

```cpp
class DistanceFactor   // 名字随意，别叫 OffsetFactor 了（不再偏移）
    : public gtsam::NoiseModelFactorT<gtsam::Vector1, gtsam::Point2, gtsam::Point2> {
public:
  using Base = gtsam::NoiseModelFactorT<gtsam::Vector1, gtsam::Point2, gtsam::Point2>;
  using Base::evaluateError;   // 你原来这行保留，必须的

  DistanceFactor(gtsam::Key fromKey, gtsam::Key toKey, double measured,
                 const gtsam::SharedNoiseModel &noiseModel)
      : Base(noiseModel, fromKey, toKey), measured_(measured) {}

  gtsam::Vector1 evaluateError(const gtsam::Point2 &p_from,
                               const gtsam::Point2 &p_to,
                               gtsam::Matrix *H_from = nullptr,
                               gtsam::Matrix *H_to = nullptr) const override {
    const gtsam::Vector2 delta = p_to - p_from;
    const double r = delta.norm();

    if (H_from || H_to) {
      gtsam::Matrix12 J;                       // ∂‖Δ‖/∂Δ ：1x2
      if (r > kEpsilon) {
        J << delta.x() / r, delta.y() / r;     // 单位向量（行）
      } else {
        J.setZero();                           // 两点重合：方向无定义（和 RangeFactor 一样）
      }
      if (H_from) *H_from = -J;                // 1x2
      if (H_to)   *H_to   =  J;                // 1x2
    }

    gtsam::Vector1 e;
    e << r - measured_;
    return e;
  }
  // clone / print 照抄原来的，measured_ 改成 double，measured() 返回 double
private:
  static constexpr double kEpsilon = 1e-9;
  double measured_;
};
```

三个容易踩的点，都在上面这段里解决了：

1. **`H` 必须是 `dim(e) × dim(p)` = `1×2`**，不是 `2×2`。你现在的版本把残差改成标量了但 `H` 还写 `Matrix2`，`checkJacobian` 会直接判 `inf`（它先比形状），而且在 release 下 `linearize()` 里做 `Σ^{-1/2} · A` 时维度不匹配，是断言/未定义行为，不是「精度差一点」。
2. **`OutputVec` 不能用 `float`**（实测编译失败）：基类 `unwhitenedError` 的返回类型硬编码是 `gtsam::Vector`，`return evaluateError(...)` 需要从 `float` 隐式构造 `Eigen::Matrix<double,-1,1>`，而 Eigen 的标量构造是 `explicit` → 报错。用 `gtsam::Vector1`（`traits<float>` 虽然存在，但解决不了这个转换问题）。
3. **噪声模型必须是 1 维**：`noiseModel::Isotropic::Sigma(1, kOffsetSigma)`。`NoiseModelFactor::dim()` 返回的是**噪声模型的维度**，残差 1 维、噪声 2 维会在 `whiten()` 里炸掉。所以原来 `offsetNoise` 与 `priorNoise` 共用 `Sigma(2, …)` 的写法必须拆开。

## 3. 为什么是这样（三条理由）

**(a) 纯链式法则。** $e$ 只通过 $r=\|\Delta\|$ 依赖位置，而范数对向量的梯度就是「沿 $\Delta$ 方向的单位向量」$\Delta/\|\Delta\|$；再乘 $\partial\Delta/\partial p = \pm I$，就是 ±单位行向量。

**(b) 几何上：距离只对「沿连线方向」的运动敏感。** 把 $p_{to}$ 沿垂直于连线的方向挪一下，$r$ 的一阶变化是 0。所以这一行雅可比的**模长恒为 1**（不管距离多远），信息量完全由白化权重 $1/\sigma$ 提供：白化后是 $w^2\hat u\hat u^{\mathsf T}$，秩 1，trace $= 1/\sigma^2$ = 400（对应我实测的特征值和恰好 1200 = 3×400）。对比原来的偏移因子 $H=\pm I$，两行范数都是 1 且**方向固定**，秩 2 —— 这就是 TODO 里「距离丢掉方向信息，秩更低」的确切含义。

**(c) 它是**非线性**的，随线性化点变化。** 所以：① 不能在 `evaluateError` 里用 `z` 或真值算方向，必须用**当前估计**的 $p_{to}-p_{from}$；② 第 2 节的自检探针必须取「误差非零、两点不重合」的点（现有代码 `truth + (0.3,-0.2)` 之类就够），否则 $r=0$ 时方向退化、或探针落在解析解上掩盖错误；③ 初值离真值远时 GN 可能跑飞，LM 更稳。

## 4. TODO1 不是只改 `evaluateError`，还要一起改这些（清单）

| 位置（当前行号） | 现在 | 要改成 |
| --- | --- | --- |
| 128/130/138-140/142-144/164/167/170 | `float` / `Vector2 measured_` | `gtsam::Vector1` 输出 + `double measured_` |
| 184-190 `Scenario` | `std::vector<Vector2> offsets` | `std::vector<double> ranges` |
| 196-199 `makeScenario` | `z = truth[j]-truth[i] + noise` | `push_back((truth[j]-truth[i]).norm() + noise(rng))` |
| 215 `buildGraph` | 传 `s.offsets[k]` | 传 `s.ranges[k]` |
| 252 | `Isotropic::Sigma(2, kOffsetSigma)` | `Sigma(1, kOffsetSigma)`（先验仍是 `Sigma(2, kPriorSigma)`） |
| 258-276 第 1 节 | 打印 2×2 的 ±I | 打印 1×2 的 `-uᵀ / +uᵀ`，`whiten(e)` 变成 1 维 |
| 312-316 一致性检查 | `z02-(z01+z12)`（对距离无意义） | 三角不等式：`ranges[0]+ranges[1]-ranges[2] ≥ 0` |
| 305 / 335 初值 | `makeInitial(s, shift)` 纯平移 | 见下面 ⚠️ |

⚠️ **一个必须注意的副作用**（我实测过）：`makeInitial` 只做整体平移时，所有距离残差**恒等于 0**（`error ≈ 1e-29`），第 3 节「初值不准 → LM 拉回来」和第 5 节「两个初值 → 两个解」都会变成「什么都没发生」。建议给 `makeInitial` 加一个尺度扰动，例如：

```cpp
Values makeInitial(const Scenario &s, const Point2 &shift, double scale = 1.0) {
  Values initial;
  for (size_t i = 0; i < kNumPoints; ++i) {
    const Point2 p = Point2(scale * s.truth[i].x(), scale * s.truth[i].y());
    initial.insert(P(i), Point2(p + shift));   // scale=1.2 → 三角形被拉大，残差非零
  }
  return initial;
}
```
（`truth[0]=(0,0)`，所以按原点缩放时 P0 不动，结果好解读。）

## 5. 改成距离因子后第 5 节该期待什么（已实测）

- 去掉先验：$6\times6$ Hessian，**nullity = 3** —— 2 个全局平移 + **1 个全局旋转**（距离对刚体运动不敏感），rank = 3（3 条边各贡献 1 维）。
- 保留 P0 先验：**nullity = 1**，只剩「绕 P0 旋转」不可观 —— 这正好可以做成 TODO1 的第二个实验，说明「加先验只钉住了平移，没钉住旋转」。
- 所以第 5 节现有那句「恰好 2 个不可观方向」的文案要改成 3，并且两个不同初值的解除了平移差之外还多一个旋转自由度（`P1-P0` 之差不再为 0）。

## 6. 建议的收尾

如果你希望我直接把这套改动落到 `src/lessons/lesson0_template.cpp`（并重跑 `xmake run lesson0` 确认第 2 节三处 `[ OK ]`、第 5 节 `nullity = 3`），请**切到 Act 模式**，我再动手。也可以只改因子类（保留 2 维偏移版本作为对照，另加一个 `DistanceFactor`），你选一个：

- 方案 A：直接把 `OffsetFactor` 改造成距离因子（改动最小，正合 TODO1 原文）。
- 方案 B：保留 `OffsetFactor`，新增 `DistanceFactor` 并在第 5 节对比两者 nullity=2 vs 3（教学效果更好，但改的地方更多）。