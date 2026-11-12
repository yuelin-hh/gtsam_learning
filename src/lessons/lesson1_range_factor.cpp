/**
 * Lesson 1：你的第一个自定义因子 —— 一维测距因子 RangeFactor
 *
 * 学习目标：
 *   1. 认识 NoiseModelFactorT<Vector, Pose2, Point2> 的骨架（构造 / evaluateError / clone）
 *   2. 亲手推导解析雅可比，并用中心差分做自检
 *   3. 看到"雅可比写错"会如何在数值自检里立刻暴露
 *   4. 一个反直觉但极重要的事实：测距因子对姿态 θ 不可观（距离与旋转无关）
 *      → 用 bearing 因子补上这个自由度，理解"因子图设计 = 信息互补"
 *
 * 运行：xmake run lesson1
 *
 * TODO 练习：
 *   1. 把 RangeFactor 的 measured_ 换成 Point2 观测量（bearing-range 因子），
 *      即 e = [bearing(T,p) - z_b; ||l|| - z_r]，写出 2x3 / 2x2 的雅可比并自检。
 *   2. 在第 3 节里把 rangeNoise 换成 noiseModel::Robust::Create(
 *      noiseModel::mEstimator::Cauchy::Create(1.0), rangeNoise)，观察 θ 的
 *      边缘方差是否还是 0.25（提示：鲁棒核不改变可观性，只改变权重）。
 */

#include "factors/range_factor.hpp"
#include "lesson_utils.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/sam/BearingFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <Eigen/Eigenvalues>

#include <iostream>
#include <vector>

using namespace gtsam;
using gtsam_learning::RangeFactor;

namespace {

using symbol_shorthand::L;
using symbol_shorthand::X;

/// 故意写错的雅可比：把"对点"的导数当成了"对位姿"的导数
class BuggyRangeFactor : public RangeFactor {
 public:
  using RangeFactor::RangeFactor;

  Vector evaluateError(const Pose2& T, const Point2& p, Matrix* H_T,
                       Matrix* H_p) const override {
    Matrix23 D_T;
    Matrix22 D_p;
    const Point2 l = T.transformTo(p, H_T ? &D_T : nullptr, H_p ? &D_p : nullptr);
    const double r = l.norm();

    Vector1 e;
    e << r - measured();

    if (H_T || H_p) {
      // BUG：用"世界坐标系"下的方向 (p - t) 代替"机体系"下的方向 l
      const Point2 l_world = p - T.translation();
      const double rw = l_world.norm();
      Matrix12 J;
      if (rw > 1e-9) {
        J << l_world.x() / rw, l_world.y() / rw;
      } else {
        J.setZero();
      }
      if (H_T) *H_T = J * D_T;  // 维度对，但数值错
      if (H_p) *H_p = J * D_p;
    }
    return e;
  }
};

struct Scenario {
  Pose2 truth{3.0, 4.0, 0.6};
  std::vector<Point2> anchors{{0.0, 0.0}, {10.0, 0.0}, {0.0, 10.0}};
};

struct GraphOptions {
  bool with_pose_prior = true;
  bool with_bearing = false;
  bool buggy = false;
};

/// 构造：路标先验 + 3 个测距因子（可选位姿先验 / 方位角因子）
NonlinearFactorGraph buildGraph(const Scenario& s,
                                const SharedNoiseModel& anchorPriorNoise,
                                const SharedNoiseModel& rangeNoise,
                                const SharedNoiseModel& bearingNoise,
                                const GraphOptions& opts = GraphOptions()) {
  NonlinearFactorGraph graph;

  if (opts.with_pose_prior) {
    // 注意：这个先验只是"弱约束"，真正的信息来自测距 / 方位角因子
    graph.emplace_shared<PriorFactor<Pose2>>(
        X(0), Pose2(0, 0, 0),
        noiseModel::Diagonal::Sigmas(Vector3(1.0, 1.0, 0.5)));
  }

  for (size_t i = 0; i < s.anchors.size(); ++i) {
    // 路标是已知常量：用极强先验"钉死"
    graph.emplace_shared<PriorFactor<Point2>>(L(i), s.anchors[i],
                                              anchorPriorNoise);
    const double z = s.truth.transformTo(s.anchors[i]).norm();
    if (opts.buggy) {
      graph.emplace_shared<BuggyRangeFactor>(X(0), L(i), z, rangeNoise);
    } else {
      graph.emplace_shared<RangeFactor>(X(0), L(i), z, rangeNoise);
    }
  }

  if (opts.with_bearing) {
    // 内置因子：测量的是 Rot2（机体系下的方位角），1 维残差
    const Rot2 z = s.truth.bearing(s.anchors[0]);
    graph.emplace_shared<BearingFactor<Pose2, Point2>>(X(0), L(0), z,
                                                       bearingNoise);
  }
  return graph;
}

Values makeInitial(const Scenario& s) {
  Values initial;
  // 别让初值正好落在锚点 (0,0) 上：那种情形距离为 0，∂‖l‖/∂l 退化
  initial.insert(X(0), Pose2(1.0, -1.0, 0.3));
  for (size_t i = 0; i < s.anchors.size(); ++i) {
    initial.insert(L(i), s.anchors[i]);
  }
  return initial;
}

void printPoseComparison(const Pose2& est, const Pose2& truth) {
  std::cout << "  真值   : " << truth.translation().transpose()
            << "  theta=" << truth.theta() << "\n  估计值 : "
            << est.translation().transpose() << "  theta=" << est.theta()
            << "\n  误差   : " << lesson::poseError(est, truth).transpose()
            << std::endl;
}

/// 优化器停止条件：只走一步（用于对比不同雅可比的下降量）
GaussNewtonParams oneStepParams() {
  GaussNewtonParams p;
  p.maxIterations = 1;
  p.errorTol = 0.0;          // error > 0 时不会提前退出
  p.absoluteErrorTol = 0.0;
  p.relativeErrorTol = 0.0;
  return p;
}

}  // namespace

int main() {
  const Scenario s;
  const Values initial = makeInitial(s);

  const auto anchorPriorNoise = noiseModel::Isotropic::Sigma(2, 1e-2);
  const auto rangeNoise = noiseModel::Isotropic::Sigma(1, 1e-2);
  const auto bearingNoise = noiseModel::Isotropic::Sigma(1, 1e-2);

  lesson::section("1. 手写雅可比 vs 数值雅可比");
  {
    const auto graph = buildGraph(s, anchorPriorNoise, rangeNoise, bearingNoise);
    // 因子图里混着先验因子和测距因子，用 RTTI 把 RangeFactor 挑出来
    for (size_t i = 0; i < graph.size(); ++i) {
      if (const auto factor =
              std::dynamic_pointer_cast<RangeFactor>(graph[i])) {
        lesson::printCheck("RangeFactor(X0,L" +
                               std::to_string(symbolIndex(factor->key<2>())) + ")",
                           lesson::checkJacobian(*factor, initial));
      }
    }
  }

  lesson::section("2. 故意写错的雅可比会被立刻抓出来");
  {
    GraphOptions opts;
    opts.buggy = true;
    const auto bad =
        buildGraph(s, anchorPriorNoise, rangeNoise, bearingNoise, opts);
    for (size_t i = 0; i < bad.size(); ++i) {
      if (const auto factor = std::dynamic_pointer_cast<RangeFactor>(bad[i])) {
        lesson::printCheck("BuggyRangeFactor(X0,L" +
                               std::to_string(symbolIndex(factor->key<2>())) + ")",
                           lesson::checkJacobian(*factor, initial));
      }
    }

    // 只走一步 GN，看错误雅可比把状态带偏多少
    const auto good = buildGraph(s, anchorPriorNoise, rangeNoise, bearingNoise);

    const Values goodStep =
        GaussNewtonOptimizer(good, initial, oneStepParams()).optimize();
    const Values badStep =
        GaussNewtonOptimizer(bad, initial, oneStepParams()).optimize();

    std::cout << "\n一步 GN 之后的误差（越小越好）:\n"
              << "  正确雅可比: graph.error = " << good.error(goodStep)
              << "\n  错误雅可比: graph.error = " << bad.error(badStep)
              << std::endl;
  }

  lesson::section("3. 测距因子只约束平移，姿态 θ 不可观");
  {
    // 3a. 完全不加位姿先验：θ 方向没有任何信息
    GraphOptions noPrior;
    noPrior.with_pose_prior = false;
    const auto free = buildGraph(s, anchorPriorNoise, rangeNoise, bearingNoise,
                                 noPrior);

    // 线性化后看 X(0) 的 Hessian 块：特征值 0 对应的就是不可观方向
    const auto linear = free.linearize(initial);
    const Matrix3 H = linear->hessianBlockDiagonal().at(X(0));
    Eigen::SelfAdjointEigenSolver<Matrix3> eig(H);
    std::cout << "  H(X0) (信息矩阵) =\n" << H << "\n  特征值 = "
              << eig.eigenvalues().transpose()
              << "\n  → 最小特征值 0，特征向量 = "
              << eig.eigenvectors().col(0).transpose()
              << "，即 θ 方向没有信息。" << std::endl;

    // 从两个不同的 θ 出发，代价函数完全相同 → 两个不同的"最优解"
    Values initA = initial;
    Values initB = initial;
    initB.update(X(0), Pose2(1.0, -1.0, -0.9));

    const Values resA = LevenbergMarquardtOptimizer(free, initA).optimize();
    const Values resB = LevenbergMarquardtOptimizer(free, initB).optimize();

    std::cout << "\n  用两个不同的 θ 初值优化（代价函数对 θ 完全不变）:"
              << "\n  初值 θ= 0.3 → 估计 " << resA.at<Pose2>(X(0))
              << ", error=" << free.error(resA)
              << "\n  初值 θ=-0.9 → 估计 " << resB.at<Pose2>(X(0))
              << ", error=" << free.error(resB) << std::endl;
  }

  lesson::section("3b. 加先验后能解，但 θ 只是在『重复先验』");
  {
    const auto graph = buildGraph(s, anchorPriorNoise, rangeNoise, bearingNoise);
    const Values result =
        LevenbergMarquardtOptimizer(graph, initial).optimize();
    printPoseComparison(result.at<Pose2>(X(0)), s.truth);

    const Marginals marginals(graph, result);
    const Matrix3 cov = marginals.marginalCovariance(X(0));
    std::cout << "\n  边缘协方差 Cov(X0) [vx, vy, omega]:\n"
              << cov << "\n  → omega 的方差 = " << cov(2, 2)
              << "，正好等于先验 sigma^2 = 0.25；\n"
              << "    说明测距因子没有给 θ 提供任何信息。" << std::endl;
  }

  lesson::section("4. 加一个 bearing 因子（内置），θ 立刻变可观");
  {
    GraphOptions withBearing;
    withBearing.with_bearing = true;
    const auto graph =
        buildGraph(s, anchorPriorNoise, rangeNoise, bearingNoise, withBearing);
    const Values result =
        LevenbergMarquardtOptimizer(graph, initial).optimize();
    printPoseComparison(result.at<Pose2>(X(0)), s.truth);

    const Marginals marginals(graph, result);
    std::cout << "\n  边缘协方差 Cov(X0):\n"
              << marginals.marginalCovariance(X(0)) << std::endl;
  }

  lesson::section("5. 残差表：\"残差大\"不等于\"影响大\"");
  {
    const auto graph = buildGraph(s, anchorPriorNoise, rangeNoise, bearingNoise);
    const Values result =
        LevenbergMarquardtOptimizer(graph, initial).optimize();
    lesson::printFactorErrors(graph, result);
    std::cout
        << "\n  注意：位姿先验的残差最大（它的中心在原点、σ=1 m），\n"
        << "  但测距因子的 σ=0.01 m，权重是它的 1e4 倍，所以 x,y 仍被\n"
        << "  测距拉到真值附近。判断「谁在拉扯状态」要同时看残差和噪声模型，\n"
        << "  只看 error 数值会被误导。" << std::endl;
  }

  return 0;
}
