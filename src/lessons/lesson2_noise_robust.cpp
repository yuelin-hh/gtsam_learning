/**
 * Lesson 2：噪声模型与鲁棒核 —— 让因子"知道"自己有多可信
 *
 * 学习目标：
 *   1. Isotropic / Diagonal / Robust 三种噪声模型的适用场景与 API
 *   2. 用 Cauchy / Huber 鲁棒核抵抗测距外点，并观察每个测量的等效权重
 *   3. Diagonal 噪声 = 每个轴独立的可信度；它在融合时会决定"听谁的"
 *
 * 运行：xmake run lesson2
 *
 * TODO 练习：
 *   1. 把 Cauchy 的 k 改成 0.3 / 5.0，观察权重和估计结果如何变化
 *      （k 越小越"狠"，大残差会被压得更低）。
 *   2. 在 Part B 里把 GpsPointFactor 换成带相关性的噪声
 *      （noiseModel::Gaussian::Covariance(2x2 非对角矩阵)），
 *      观察估计值是否还落在两个观测的"逐轴加权平均"上。
 */

#include "factors/gps_point_factor.hpp"
#include "factors/range_factor.hpp"
#include "lesson_utils.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <set>
#include <string>
#include <vector>

using namespace gtsam;
using gtsam_learning::GpsPointFactor;
using gtsam_learning::RangeFactor;

namespace {

using symbol_shorthand::L;
using symbol_shorthand::X;

// ---------------------------------------------------------------------------
// Part A：带外点的测距 + 鲁棒核
// ---------------------------------------------------------------------------

struct RobustScenario {
  Pose2 truth{2.0, 1.0, 0.3};
  std::vector<Point2> anchors{{0.0, 0.0},  {8.0, 0.0},  {0.0, 8.0}, {8.0, 8.0},
                              {4.0, -6.0}, {-5.0, 3.0}, {12.0, 4.0}};
  std::vector<double> ranges;
  std::set<int> outliers{1, 3}; ///< 第 1、3 个测量被人为污染
};

RobustScenario makeRobustScenario() {
  RobustScenario s;
  for (size_t i = 0; i < s.anchors.size(); ++i) {
    double z = s.truth.transformTo(s.anchors[i]).norm();
    // 外点：比真值多 2.5 m（模拟多径 / 遮挡）
    if (s.outliers.count(static_cast<int>(i)))
      z += 2.5;
    s.ranges.push_back(z);
  }
  return s;
}

NonlinearFactorGraph buildRangeGraph(const RobustScenario &s,
                                     const SharedNoiseModel &rangeNoise,
                                     const SharedNoiseModel &anchorNoise) {
  NonlinearFactorGraph graph;
  // 罗盘先验：θ 很准，x/y 几乎不管（σ=10 m）
  graph.emplace_shared<PriorFactor<Pose2>>(
      X(0), Pose2(0.0, 0.0, s.truth.theta()),
      noiseModel::Diagonal::Sigmas(Vector3(10.0, 10.0, 0.01)));

  for (size_t i = 0; i < s.anchors.size(); ++i) {
    graph.emplace_shared<PriorFactor<Point2>>(L(i), s.anchors[i], anchorNoise);
    graph.emplace_shared<RangeFactor>(X(0), L(i), s.ranges[i], rangeNoise);
  }
  return graph;
}

Values makeInitial(const RobustScenario &s) {
  Values initial;
  initial.insert(X(0), Pose2(0.0, 0.0, s.truth.theta()));
  for (size_t i = 0; i < s.anchors.size(); ++i)
    initial.insert(L(i), s.anchors[i]);
  return initial;
}

/// 打印每个测距的残差与鲁棒核给出的等效权重（非鲁棒模型权重恒为 1）
void printRangeResiduals(const NonlinearFactorGraph &graph,
                         const Values &values) {
  size_t idx = 0;
  for (size_t i = 0; i < graph.size(); ++i) {
    const auto factor = std::dynamic_pointer_cast<RangeFactor>(graph[i]);
    if (!factor)
      continue;

    const Vector e = factor->unwhitenedError(values);
    double weight = 1.0;
    if (const auto robust = std::dynamic_pointer_cast<noiseModel::Robust>(
            factor->noiseModel())) {
      // 先算内层高斯模型下的马氏距离，再问鲁棒核"这个距离该打几折"
      const double d = robust->noise()->mahalanobisDistance(e);
      weight = robust->robust()->weight(d);
    }
    std::cout << "    z[" << idx++ << "] |e|=" << std::fixed
              << std::setprecision(3) << std::abs(e(0))
              << " m,  weight=" << weight << "\n";
  }
}

} // namespace

int main() {
  const RobustScenario s = makeRobustScenario();
  const auto gaussian = noiseModel::Isotropic::Sigma(1, 0.1);
  const auto anchorNoise = noiseModel::Isotropic::Sigma(2, 1e-3);

  lesson::section("1. 同一个图、同一组测量，三种噪声模型");
  std::cout << "真值: x=" << s.truth.x() << ", y=" << s.truth.y()
            << "\n外点索引: ";
  for (int i : s.outliers)
    std::cout << i << " ";
  std::cout << "（每个读数 +2.5 m）\n\n";

  std::cout << std::left << std::setw(20) << "噪声模型" << std::right
            << std::setw(10) << "est_x" << std::setw(10) << "est_y"
            << std::setw(12) << "xy_err" << std::setw(14) << "cost\n";

  struct ModelSpec {
    std::string name;
    SharedNoiseModel noise;
  };
  const auto cauchyNoise = noiseModel::Robust::Create(
      noiseModel::mEstimator::Cauchy::Create(1.0), gaussian);
  const auto huberNoise = noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(1.345), gaussian);
  const std::vector<ModelSpec> models{{"Gaussian(sigma=0.1)", gaussian},
                                      {"Cauchy(k=1.0)", cauchyNoise},
                                      {"Huber(k=1.345)", huberNoise}};

  Values huberResult;
  Values cauchyResult;
  for (const auto &m : models) {
    const auto graph = buildRangeGraph(s, m.noise, anchorNoise);
    const Values result =
        LevenbergMarquardtOptimizer(graph, makeInitial(s)).optimize();
    if (m.name.rfind("Huber", 0) == 0)
      huberResult = result;
    if (m.name.rfind("Cauchy", 0) == 0)
      cauchyResult = result;
    const Pose2 est = result.at<Pose2>(X(0));
    const double xy = std::hypot(est.x() - s.truth.x(), est.y() - s.truth.y());

    std::cout << std::left << std::setw(20) << m.name << std::right
              << std::fixed << std::setprecision(4) << std::setw(10) << est.x()
              << std::setw(10) << est.y() << std::setw(12) << xy
              << std::setw(14) << graph.error(result) << "\n";
  }

  {
    const auto cauchyGraph = buildRangeGraph(s, cauchyNoise, anchorNoise);
    std::cout << "\n  Cauchy（从 (0,0) 出发）的逐测量权重:\n";
    printRangeResiduals(cauchyGraph, cauchyResult);
    std::cout << "  → 它的解是错误的局部极小：多数内点被当成外点压掉，\n"
                 "    而 2 个外点反而拿到了不小的权重。\n";
  }

  // 鲁棒代价是非凸的：Cauchy 从 (0,0) 出发会掉进局部极小，
  // 从 Huber 的解出发（先粗后精 / GNC 的思路）才能到全局最优。
  {
    const auto cauchyGraph = buildRangeGraph(s, cauchyNoise, anchorNoise);
    const Values result =
        LevenbergMarquardtOptimizer(cauchyGraph, huberResult).optimize();
    const Pose2 est = result.at<Pose2>(X(0));
    std::cout << "\n  用 Huber 的解作为 Cauchy 的初值再优化一次:\n"
              << "    estimate=(" << est.x() << ", " << est.y() << "), xy_err="
              << std::hypot(est.x() - s.truth.x(), est.y() - s.truth.y())
              << "\n    逐测量权重（应只剩 2 个外点被压制）:\n";
    printRangeResiduals(cauchyGraph, result);
    std::cout << "  → 鲁棒核的代价不是凸的，初值仍然重要；工程上常用"
                 "「大 k 起步、逐步收紧」或 GNC。\n";
  }

  lesson::section("2. Diagonal 噪声：每个轴有自己的可信度");
  {
    // 两次 GPS 观测，故意让"谁更可信"在 x / y 上相反
    const Point2 z1(3.0, 4.0);
    const Point2 z2(3.4, 3.6);

    struct GpsCase {
      std::string name;
      Vector2 sigma1;
      Vector2 sigma2;
    };
    const std::vector<GpsCase> cases{
        {"各向异性(交换)", Vector2(0.05, 0.5), Vector2(0.5, 0.05)},
        {"各向同性 0.5", Vector2(0.5, 0.5), Vector2(0.5, 0.5)},
    };

    for (const auto &c : cases) {
      NonlinearFactorGraph graph;
      // 只约束 θ；x/y 交给 GPS
      graph.emplace_shared<PriorFactor<Pose2>>(
          X(0), Pose2(0.0, 0.0, 0.6),
          noiseModel::Diagonal::Sigmas(Vector3(10.0, 10.0, 1e-3)));
      graph.emplace_shared<GpsPointFactor>(
          X(0), z1, noiseModel::Diagonal::Sigmas(c.sigma1));
      graph.emplace_shared<GpsPointFactor>(
          X(0), z2, noiseModel::Diagonal::Sigmas(c.sigma2));

      Values init;
      init.insert(X(0), Pose2(0.0, 0.0, 0.6));
      const Values result = LevenbergMarquardtOptimizer(graph, init).optimize();
      const Pose2 est = result.at<Pose2>(X(0));
      const Matrix3 cov = Marginals(graph, result).marginalCovariance(X(0));

      std::cout << std::left << std::setw(18) << c.name << std::right
                << std::fixed << std::setprecision(4) << " estimate=("
                << est.x() << ", " << est.y() << ")" << "  Cov diag=("
                << cov(0, 0) << ", " << cov(1, 1) << ")\n";
    }
    std::cout << "\n  逐轴独立加权：z1=(3.0,4.0)、z2=(3.4,3.6)\n"
              << "  各向异性时 x 更信 z1、y 更信 z2 → (3.01, 3.60)；\n"
              << "  各向同性时两个观测等权 → (3.20, 3.80)。\n"
              << "  可见协方差不是「装饰」：它直接决定融合结果。" << std::endl;
  }

  return 0;
}
