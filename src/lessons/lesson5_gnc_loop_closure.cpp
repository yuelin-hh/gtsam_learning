/**
 * Lesson 5：错误回环（perceptual aliasing）+ GNC 逐步剔除
 *
 * 学习目标：
 *   1. 回环检测不可能 100% 正确：只要有 2 个错误回环，普通最小二乘就被拖到
 *      1.2 m 量级（真值链只有 0.13 m），整条轨迹跟着变形
 *   2. Cauchy 鲁棒核能把解救回来，但它给出的权重是"软"的（连续值，约 0.1），
 *      外点权重大小取决于残差 / 核参数 c 的比例 —— 需要调参，
 *      而且中等大小的外点会残留可观的影响
 *   3. GNC（Graduated Non-Convexity）：先解一个接近最小二乘的凸问题，
 *      再逐步收紧到截断最小二乘（TLS），权重最终收敛到严格的 0/1 ——
 *      既能给出硬判决（可直接当外点分类器用），对外点尺度也不敏感
 *   4. 用 params.setKnownInliers 告诉 GNC「里程计都可信」，
 *      只对回环做外点剔除 —— 工程上的标准做法
 *   5. GNC 的局限：它找的是「内点最多的自洽集合」，而不是「真值」。
 *      当错误回环彼此自洽（同一个偏置 ⇒ 等价于轨迹整体形变）且数量占优时，
 *      GNC 会把内点全判成外点（第 5 节实测：3 个真回环全被丢掉）。
 *      软的 Cauchy 反而更稳 → 前端需要 PCM / 局部一致性检验，两者互补
 *
 * 运行：xmake run lesson5
 *
 * TODO 练习：
 *   1. 把 setKnownInliers 去掉，看看里程计因子会不会被误判成外点。
 *   2. 把 lossType 从 TLS 换成 GM（Geman-McClure），比较收敛速度与最终权重。
 *   3. 用 params.setInlierCostThresholds 手动改内点阈值，观察分类结果如何变化。
 */

#include "lesson_utils.hpp"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/GncParams.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <utility>
#include <vector>

using namespace gtsam;

namespace {

using symbol_shorthand::X;

constexpr size_t kN = 12;

/// 备用回环对（用来凑出不同数量的错误回环）
const std::vector<std::pair<size_t, size_t>> kFalsePairs{
    {0, 11}, {4, 7}, {0, 9}, {2, 7}, {3, 10}, {5, 6}};

/// 一条回环约束
struct LoopClosure {
  size_t i;
  size_t j;
  Pose2 z;
  bool is_outlier{false};  ///< 只用于事后打分，优化器并不知道
};

struct Data {
  std::vector<Pose2> truth;
  std::vector<Pose2> odometry;  ///< size = kN - 1
  std::vector<LoopClosure> loops;
};

/**
 * 场景：机器人沿走廊开出去再原路返回。
 *   去程 X0..X5 在 x = 0..5，朝向 +x；
 *   回程 X6..X11 在 x = 5..0，朝向 -x。
 * 所以 (1,10)、(2,9)、(3,8) 是真的又见到了，正确；
 * 其它回环是感知混叠 —— 长得像，其实偏了 offset 米。
 */
Data makeData(size_t n_false, double offset, bool consistent_outliers,
              unsigned seed = 11) {
  Data d;
  for (size_t i = 0; i < 6; ++i) {
    d.truth.emplace_back(static_cast<double>(i), 0.0, 0.0);
  }
  for (size_t i = 6; i < kN; ++i) {
    d.truth.emplace_back(11.0 - static_cast<double>(i), 0.0, M_PI);
  }

  std::mt19937 rng(seed);
  std::normal_distribution<double> odom_noise(0.0, 0.1);
  std::normal_distribution<double> loop_noise(0.0, 0.03);

  for (size_t i = 0; i + 1 < kN; ++i) {
    d.odometry.push_back(
        d.truth[i].between(d.truth[i + 1]) *
        Pose2(odom_noise(rng), odom_noise(rng), 0.05 * odom_noise(rng)));
  }

  for (const auto& [i, j] :
       std::vector<std::pair<size_t, size_t>>{{1, 10}, {2, 9}, {3, 8}}) {
    d.loops.push_back(
        {i, j,
         d.truth[i].between(d.truth[j]) *
             Pose2(loop_noise(rng), loop_noise(rng), 0.1 * loop_noise(rng)),
         false});
  }
  for (size_t k = 0; k < n_false; ++k) {
    const auto [i, j] = kFalsePairs[k];
    // 感知混叠：相对位姿被整体偏置 offset 米
    //   consistent_outliers = false：每个错回环偏的方向/大小都不一样
    //   consistent_outliers = true ：所有错回环共用同一个偏置，于是它们
    //   「彼此自洽」—— 等价于把整条轨迹做了一次形变，最难分辨（见第 5 节）
    const Pose2 bias =
        consistent_outliers
            ? Pose2(offset, 0.5 * offset, 0.05 * offset)
            : Pose2(offset * (1.0 + 0.4 * static_cast<double>(k)),
                    0.5 * offset * (k % 2 ? 1.0 : -1.0), 0.05 * offset);
    d.loops.push_back({i, j, d.truth[i].between(d.truth[j]) * bias, true});
  }
  return d;
}

const auto kOdomNoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.02));
const auto kLoopNoise = noiseModel::Diagonal::Sigmas(Vector3(0.05, 0.05, 0.02));

/// 返回图；loop_indices 给出各条回环在因子图中的下标（GNC 的权重按因子下标给）
NonlinearFactorGraph buildGraph(const Data& d, bool robust_loops,
                                std::vector<size_t>* loop_indices) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose2>>(
      X(0), d.truth[0], noiseModel::Isotropic::Sigma(3, 1e-3));

  for (size_t i = 0; i + 1 < kN; ++i) {
    graph.emplace_shared<BetweenFactor<Pose2>>(X(i), X(i + 1), d.odometry[i],
                                               kOdomNoise);
  }

  const SharedNoiseModel loop_model =
      robust_loops
          ? SharedNoiseModel(noiseModel::Robust::Create(
                noiseModel::mEstimator::Cauchy::Create(1.0), kLoopNoise))
          : SharedNoiseModel(kLoopNoise);
  for (const auto& lc : d.loops) {
    if (loop_indices) loop_indices->push_back(graph.size());
    graph.emplace_shared<BetweenFactor<Pose2>>(X(lc.i), X(lc.j), lc.z,
                                              loop_model);
  }
  return graph;
}

/// 死推初值（X0 用真值，因为它有先验）
Values initialGuess(const Data& d) {
  Values initial;
  Pose2 pose = d.truth[0];
  initial.insert(X(0), pose);
  for (size_t i = 0; i + 1 < kN; ++i) {
    pose = pose * d.odometry[i];
    initial.insert(X(i + 1), pose);
  }
  return initial;
}

std::vector<Pose2> toVector(const Values& v) {
  std::vector<Pose2> out;
  for (size_t i = 0; i < kN; ++i) out.push_back(v.at<Pose2>(X(i)));
  return out;
}

std::pair<double, double> rmse(const std::vector<Pose2>& est, const Data& d) {
  double sum_xy = 0.0;
  double sum_th = 0.0;
  for (size_t i = 0; i < kN; ++i) {
    const Vector3 e = lesson::poseError(est[i], d.truth[i]);
    sum_xy += e.head<2>().squaredNorm();
    sum_th += e.z() * e.z();
  }
  return {std::sqrt(sum_xy / kN), std::sqrt(sum_th / kN)};
}

/// 打印一行结论（RMSE + 图的总误差）
void report(const std::string& name, const Values& result, const Data& d,
            const NonlinearFactorGraph& graph) {
  const auto [xy, th] = rmse(toVector(result), d);
  std::cout << "  " << std::left << std::setw(20) << name << std::right
            << "xy_RMSE=" << std::fixed << std::setprecision(4) << xy
            << " m,  theta_RMSE=" << std::setprecision(4) << th << " rad\n";
  std::cout.unsetf(std::ios::fixed);
}

/// 三种解法的结果（同一个初值、同一份数据）
struct Results {
  Values plain;
  Values cauchy;
  Values gnc;
  Vector cauchy_weights;             ///< 每条回环的 Cauchy 软权重
  Vector gnc_weights;                ///< 每个因子的 GNC 硬权重
  std::vector<size_t> loop_indices;  ///< 回环在因子图中的下标
};

/// 依次跑：普通 LM、LM + Cauchy、GNC(TLS, 里程计已知内点)
Results solveAll(const Data& d) {
  Results r;
  const Values init = initialGuess(d);

  const auto g_plain = buildGraph(d, false, &r.loop_indices);
  r.plain = LevenbergMarquardtOptimizer(g_plain, init).optimize();

  const auto g_cauchy = buildGraph(d, true, nullptr);
  r.cauchy = LevenbergMarquardtOptimizer(g_cauchy, init).optimize();

  // 逐条回环读出 Cauchy 的实际权重：Robust::weight 内部会用内层高斯模型
  // 算马氏距离，所以这里要传「未白化」的残差
  r.cauchy_weights.resize(d.loops.size());
  for (size_t k = 0; k < d.loops.size(); ++k) {
    const auto factor = std::dynamic_pointer_cast<NoiseModelFactor>(
        g_cauchy.at(r.loop_indices[k]));
    const auto robust =
        std::dynamic_pointer_cast<noiseModel::Robust>(factor->noiseModel());
    r.cauchy_weights(k) =
        robust->weight(factor->unwhitenedError(r.cauchy));
  }

  GncParams<LevenbergMarquardtParams> params;
  params.setLossType(TLS);
  params.setVerbosityGNC(GncParams<LevenbergMarquardtParams>::SILENT);
  params.baseOptimizerParams.setMaxIterations(100);

  GncParams<LevenbergMarquardtParams>::IndexVector known_inliers;
  for (size_t i = 0; i < kN; ++i) {
    known_inliers.push_back(i);  // 0 = 先验, 1..11 = 里程计
  }
  params.setKnownInliers(known_inliers);

  GncOptimizer<GncParams<LevenbergMarquardtParams>> gnc(g_plain, init, params);
  r.gnc = gnc.optimize();
  r.gnc_weights = gnc.getWeights();
  return r;
}

/// GNC 权重 > 0.5 视作内点，统计分类正确率
size_t countCorrectClassification(const Data& d, const Results& r) {
  size_t correct = 0;
  for (size_t k = 0; k < d.loops.size(); ++k) {
    const bool predicted_inlier = r.gnc_weights(r.loop_indices[k]) > 0.5;
    if (predicted_inlier != d.loops[k].is_outlier) ++correct;
  }
  return correct;
}

}  // namespace

int main() {
  const Data d = makeData(/*n_false=*/2, /*offset=*/2.0, /*consistent=*/false);

  lesson::section("0. 场景：去程 6 帧 + 回程 6 帧，5 条回环里有 2 条是错的");
  {
    std::cout << "  回环      测量 (x, y, th)        真值 (x, y, th)      类别\n";
    for (const auto& lc : d.loops) {
      const Pose2 truth = d.truth[lc.i].between(d.truth[lc.j]);
      std::cout << "  X" << std::left << std::setw(2) << lc.i << "->X"
                << std::setw(3) << lc.j << std::right << std::fixed
                << std::setprecision(2) << std::setw(6) << "(" << lc.z.x()
                << "," << lc.z.y() << "," << lc.z.theta() << ")" << std::setw(8)
                << "(" << truth.x() << "," << truth.y() << "," << truth.theta()
                << ")" << std::setw(8) << (lc.is_outlier ? "错误" : "正确")
                << "\n";
    }
    std::cout.unsetf(std::ios::fixed);
    std::cout << "  （里程计 sigma = 0.1 m/步，回环 sigma = 0.05 m，"
                 "错误回环偏了 2~3 m，约 40~60 sigma）\n";
  }

  lesson::section("1. 三种处理方式对比");
  const Results r = solveAll(d);
  {
    const auto g_plain = buildGraph(d, false, nullptr);
    report("普通 LM（无鲁棒）", r.plain, d, g_plain);
    report("LM + Cauchy 核", r.cauchy, d, g_plain);
    report("GNC + TLS", r.gnc, d, g_plain);
    std::cout << "  → 普通最小二乘完全被 2 个坏点支配：RMSE 从 0.13 m 涨到 1.19 m。\n"
                 "    两种鲁棒方法都把它救回来了，差别在下一节的权重上。\n";
  }

  lesson::section("2. 每条回环的权重：Cauchy 是「软」的，GNC 是「硬」的");
  {
    std::cout << "  回环      真实   Cauchy weight   GNC weight\n";
    for (size_t k = 0; k < d.loops.size(); ++k) {
      const auto& lc = d.loops[k];
      std::cout << "  X" << std::left << std::setw(2) << lc.i << "->X"
                << std::setw(3) << lc.j << std::setw(7)
                << (lc.is_outlier ? "外点" : "内点") << std::right << std::fixed
                << std::setprecision(4) << std::setw(14)
                << r.cauchy_weights(k) << std::setw(13)
                << r.gnc_weights(r.loop_indices[k]) << "\n";
    }
    std::cout.unsetf(std::ios::fixed);
    std::cout << "  → Cauchy 权重是 1/(1+(d/c)^2)，永远大于 0，大小取决于\n"
                 "    残差 / 核参数 c 的比值：c 调小一点，内点也会被降权；\n"
                 "    c 调大一点，外点又开始污染。GNC 通过逐步收紧阈值，\n"
                 "    最终给出严格 0/1 —— 可以直接当外点分类器输出用，\n"
                 "    分类正确率 = " << countCorrectClassification(d, r) << "/"
              << d.loops.size() << "。\n";
  }

  lesson::section("3. 逐帧位置误差：外点把普通 LM 拖成什么样");
  {
    const auto plain_est = toVector(r.plain);
    const auto gnc_est = toVector(r.gnc);

    std::cout << "  位姿   普通LM误差(m)   GNC误差(m)\n";
    for (size_t i = 0; i < kN; ++i) {
      std::cout << "   X" << std::left << std::setw(4) << i << std::right
                << std::fixed << std::setprecision(4) << std::setw(12)
                << lesson::poseError(plain_est[i], d.truth[i]).head<2>().norm()
                << std::setw(12)
                << lesson::poseError(gnc_est[i], d.truth[i]).head<2>().norm()
                << "\n";
    }
    std::cout.unsetf(std::ios::fixed);
    std::cout << "  → X0 有强先验拉不回来，误差就沿里程计链向中间传播，\n"
                 "    整个闭环被拉成一条「斜」的链。\n";
  }

  lesson::section("4. 外点变「隐蔽」且变多：4 个方向各异的错回环");
  {
    const Data hard = makeData(/*n_false=*/4, /*offset=*/0.5, /*consistent=*/false);
    const Results hr = solveAll(hard);
    const auto g_plain = buildGraph(hard, false, nullptr);
    report("普通 LM", hr.plain, hard, g_plain);
    report("LM + Cauchy 核", hr.cauchy, hard, g_plain);
    report("GNC + TLS", hr.gnc, hard, g_plain);
    std::cout << "  → 外点只有 0.5 m（10 sigma）、而且方向各异。Cauchy 是「软」的，\n"
                 "    降权后仍能靠 3 个正确回环拉住解；GNC 的硬判决反而会因为\n"
                 "    内点阈值判断错一个，分类正确率 = "
              << countCorrectClassification(hard, hr) << "/" << hard.loops.size()
              << "。\n";
  }

  lesson::section("5. GNC 的软肋：4 个「彼此自洽」的错误回环");
  {
    const Data fake =
        makeData(/*n_false=*/4, /*offset=*/0.5, /*consistent=*/true);
    const Results fr = solveAll(fake);
    const auto g_plain = buildGraph(fake, false, nullptr);
    report("普通 LM", fr.plain, fake, g_plain);
    report("LM + Cauchy 核", fr.cauchy, fake, g_plain);
    report("GNC + TLS", fr.gnc, fake, g_plain);

    std::cout << "  GNC 权重（1 = 当作内点）:\n";
    for (size_t k = 0; k < fake.loops.size(); ++k) {
      const auto& lc = fake.loops[k];
      std::cout << "    X" << std::left << std::setw(2) << lc.i << "->X"
                << std::setw(3) << lc.j << std::setw(7)
                << (lc.is_outlier ? "外点" : "内点") << std::right << std::fixed
                << std::setprecision(2) << fr.gnc_weights(fr.loop_indices[k])
                << "\n";
    }
    std::cout.unsetf(std::ios::fixed);
    std::cout
        << "  → 4 个错回环共用同一个偏置，于是它们互相支持，构成了另一个\n"
           "    「自洽」的解释。GNC 追求的是「内点最多的自洽集合」：\n"
           "    这里 4 个错回环比 3 个正确回环多，于是它把真的全判成外点、\n"
           "    把假的全部收下：RMSE 只做到 0.38 m，远不如 Cauchy 的 0.12 m。\n"
           "    而「软」的 Cauchy 因为外点残差大只被降到 0.1 左右，\n"
           "    靠里程计和 3 个正确回环还把解拉在真值附近。\n"
           "    → 实践上先用 PCM（成对一致性最大化）或局部一致性检验把外点\n"
           "      压成少数，再交给 GNC 精修；两者互补，不是替代关系。\n";
  }

  return 0;
}

