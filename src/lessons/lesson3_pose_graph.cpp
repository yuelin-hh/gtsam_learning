/**
 * Lesson 3：位姿图 —— 流形上的优化与增量求解
 *
 * 学习目标：
 *   1. Pose2 是流形：compose / between / retract / localCoordinates 的正确用法
 *      （为什么不能直接对 (x, y, θ) 做线性插值）
 *   2. 用 BetweenFactor<Pose2> 搭一个带里程计链 + 回环的位姿图，
 *      看清因子图的稀疏结构
 *   3. 回环把累积漂移"拉回去"：对比加回环前后的逐位姿误差
 *   4. iSAM2：增量式求解，新因子只影响与它相关的子树
 *
 * 运行：xmake run lesson3
 *
 * TODO 练习：
 *   1. 把回环因子改成 Robust(Cauchy)，再人为把回环测量偏 0.5 m，
 *      观察它是否会被自动降权（对比 L2 的权重打印方法）。
 *   2. 把里程计噪声从 0.02 调到 0.2，看回环对 X1..X4 的修正量如何变化。
 */

#include "lesson_utils.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <algorithm>

using namespace gtsam;

namespace {

using symbol_shorthand::X;

constexpr size_t kNumPoses = 6;

/// 用固定种子造一条"轻微右转"的轨迹 + 带噪声的里程计/回环测量
struct PoseGraphData {
  std::vector<Pose2> truth;
  std::vector<Pose2> odometry;  ///< odometry[i] = 测量到的 X(i) -> X(i+1)
  Pose2 loop;                   ///< 测量到的 X(N-1) -> X(0)
};

PoseGraphData makePoseGraphData() {
  PoseGraphData data;
  for (size_t i = 0; i < kNumPoses; ++i) {
    data.truth.emplace_back(1.0 * static_cast<double>(i),
                            0.05 * static_cast<double>(i),
                            0.10 * static_cast<double>(i));
  }

  std::mt19937 rng(42);
  std::normal_distribution<double> odom_noise(0.0, 0.05);
  std::normal_distribution<double> loop_noise(0.0, 0.01);
  const auto sample =
      [](std::normal_distribution<double>& dist, std::mt19937& gen) {
        return Pose2(dist(gen), dist(gen), 0.5 * dist(gen));
      };

  for (size_t i = 0; i + 1 < kNumPoses; ++i) {
    data.odometry.push_back(data.truth[i].between(data.truth[i + 1]) *
                            sample(odom_noise, rng));
  }
  data.loop = data.truth[kNumPoses - 1].between(data.truth[0]) *
              sample(loop_noise, rng);
  return data;
}

/// 搭图：先验 + 里程计链（+ 可选回环）
NonlinearFactorGraph buildPoseGraph(const PoseGraphData& data,
                                    const SharedNoiseModel& odomNoise,
                                    const SharedNoiseModel& loopNoise,
                                    bool with_loop) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose2>>(
      X(0), data.truth[0], noiseModel::Isotropic::Sigma(3, 1e-3));

  for (size_t i = 0; i + 1 < kNumPoses; ++i) {
    graph.emplace_shared<BetweenFactor<Pose2>>(X(i), X(i + 1),
                                               data.odometry[i], odomNoise);
  }
  if (with_loop) {
    graph.emplace_shared<BetweenFactor<Pose2>>(X(kNumPoses - 1), X(0),
                                               data.loop, loopNoise);
  }
  return graph;
}

/// 初值：全部放在原点（比里程计递推更"差"，更能看出优化器的作用）
Values identityInitial() {
  Values initial;
  for (size_t i = 0; i < kNumPoses; ++i) {
    initial.insert(X(i), Pose2(0.0, 0.0, 0.0));
  }
  return initial;
}

/// 打印"因子 × 变量"的稀疏结构（# 表示该因子牵涉该变量）
void printStructure(const NonlinearFactorGraph& graph) {
  std::vector<Key> keys;
  for (const auto& f : graph) {
    for (Key k : f->keys()) {
      if (std::find(keys.begin(), keys.end(), k) == keys.end()) keys.push_back(k);
    }
  }
  std::sort(keys.begin(), keys.end());

  std::cout << "  变量: ";
  for (Key k : keys) std::cout << DefaultKeyFormatter(k) << " ";
  std::cout << "\n";
  for (size_t i = 0; i < graph.size(); ++i) {
    std::cout << "  f" << std::setw(2) << i << " | ";
    for (Key k : keys) {
      const auto& ks = graph[i]->keys();
      const bool touches =
          std::find(ks.begin(), ks.end(), k) != ks.end();
      std::cout << (touches ? '#' : '.') << " ";
    }
    std::cout << "\n";
  }
}

/// 逐位姿误差表
void printPoseErrors(const Values& values, const PoseGraphData& data,
                     const std::string& title) {
  std::cout << "  " << title << "\n";
  std::cout << "   i   est_x   est_y   est_th   err_x   err_y   err_th\n";
  for (size_t i = 0; i < kNumPoses; ++i) {
    const Pose2& est = values.at<Pose2>(X(i));
    const Vector3 e = lesson::poseError(est, data.truth[i]);
    std::cout << std::fixed << std::setprecision(4) << std::setw(4) << i
              << std::setw(8) << est.x() << std::setw(8) << est.y()
              << std::setw(9) << est.theta()
              << std::setw(8) << e.x() << std::setw(8) << e.y()
              << std::setw(9) << e.z() << "\n";
  }
  std::cout.unsetf(std::ios::fixed);
}

}  // namespace

int main() {
  lesson::section("1. Pose2 是流形：compose / between / retract / local");
  {
    const Pose2 A(1.0, 2.0, 0.3);
    const Pose2 B(0.5, -0.2, 0.4);
    std::cout << "  A = " << A << "\n  B = " << B << "\n";

    std::cout << "  A * B (compose)       = " << (A * B) << "\n";
    std::cout << "  A.between(B)          = " << A.between(B) << "\n";
    std::cout << "  A.localCoordinates(B) = " << A.localCoordinates(B).transpose()
              << "\n  Logmap(A.between(B))  = "
              << Pose2::Logmap(A.between(B)).transpose()
              << "\n  → 前者是优化器实际使用的 chart（(dx,dy,dθ)，平移直接取差），\n"
                 "    后者是精确 SE(2) 指数映射（多一个 V(θ)^{-1} 修正），\n"
                 "    θ→0 时两者一致，差值是二阶小量。\n";

    const Vector3 xi(0.3, -0.2, 0.5);
    const Pose2 A_xi = A.retract(xi);
    std::cout << "\n  A.retract(" << xi.transpose() << ") = " << A_xi
              << "\n  A.localCoordinates(A.retract(xi)) = "
              << A.localCoordinates(A_xi).transpose() << "  （应等于 xi）\n";

    // 为什么优化必须在切空间做：θ 有 ±π 的环绕
    const Pose2 C(0.0, 0.0, 3.0);
    const Pose2 D(0.0, 0.0, -3.0);
    const double naive = 0.5 * (C.theta() + D.theta());
    const double manifold = C.retract(0.5 * C.localCoordinates(D)).theta();
    std::cout << "\n  θ 从 3.0 到 -3.0：\n"
              << "    朴素角度平均 = " << naive << " rad（错，方向反了 180°）\n"
              << "    流形上取中点 = " << manifold << " rad（≈ ±π，正确）\n";
  }

  const PoseGraphData data = makePoseGraphData();
  const auto odomNoise = noiseModel::Diagonal::Sigmas(Vector3(0.05, 0.05, 0.025));
  const auto loopNoise = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.005));

  lesson::section("2. 位姿图的稀疏结构");
  {
    const auto graph = buildPoseGraph(data, odomNoise, loopNoise, true);
    printStructure(graph);
    std::cout << "  → 里程计因子只连相邻两个变量（带宽很小），\n"
                 "    最后一个因子（回环）连接 X5 与 X0，会在 Hessian 里引入「填充」。\n";
  }

  lesson::section("3. 回环把累积漂移拉回去");
  {
    for (bool with_loop : {false, true}) {
      const auto graph = buildPoseGraph(data, odomNoise, loopNoise, with_loop);
      const Values init = identityInitial();
      LevenbergMarquardtOptimizer optimizer(graph, init);
      const Values result = optimizer.optimize();

      std::cout << "\n  ---- " << (with_loop ? "带回环" : "只有里程计")
                << " ----\n  iterations=" << optimizer.iterations()
                << ",  error: " << std::fixed << std::setprecision(3)
                << graph.error(init) << " -> " << graph.error(result)
                << std::defaultfloat << "\n";
      printPoseErrors(result, data, "逐位姿状态与误差 [vx, vy, ω]");
    }
  }

  lesson::section("4. iSAM2 增量求解：回环只改动受影响的子树");
  {
    ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    ISAM2 isam(params);

    // X0 的先验
    {
      NonlinearFactorGraph g;
      g.emplace_shared<PriorFactor<Pose2>>(
          X(0), data.truth[0], noiseModel::Isotropic::Sigma(3, 1e-3));
      Values v;
      v.insert(X(0), Pose2(0.0, 0.0, 0.0));
      isam.update(g, v);
    }
    // 一条一条喂里程计；新变量用里程计预测做初值（工程上的标准做法）
    for (size_t i = 0; i + 1 < kNumPoses; ++i) {
      NonlinearFactorGraph g;
      g.emplace_shared<BetweenFactor<Pose2>>(X(i), X(i + 1), data.odometry[i],
                                             odomNoise);
      Values v;
      v.insert(X(i + 1),
               isam.calculateEstimate<Pose2>(X(i)) * data.odometry[i]);
      isam.update(g, v);
    }

    const Values before = isam.calculateEstimate();
    std::cout << "  只加里程计时 X" << (kNumPoses - 1) << " = "
              << before.at<Pose2>(X(kNumPoses - 1)) << "\n";

    // 加入回环：没有新变量
    {
      NonlinearFactorGraph g;
      g.emplace_shared<BetweenFactor<Pose2>>(X(kNumPoses - 1), X(0), data.loop,
                                             loopNoise);
      isam.update(g, Values());
      // iSAM2 每次 update 只做一轮线性化+求解；回环让线性化点变化很大，
      // 多调用几次空 update（等价于再迭代几轮）让它收敛到和批量解一致。
      for (int k = 0; k < 5; ++k) isam.update();
    }
    const Values after = isam.calculateEstimate();

    std::cout << "  加入回环后 X" << (kNumPoses - 1) << " = "
              << after.at<Pose2>(X(kNumPoses - 1)) << "\n\n"
              << "  回环引起的位置修正量（m）:\n";
    for (size_t i = 0; i < kNumPoses; ++i) {
      const Point2 d = after.at<Pose2>(X(i)).translation() -
                       before.at<Pose2>(X(i)).translation();
      std::cout << "    X" << i << ": " << std::fixed << std::setprecision(4)
                << d.norm() << "\n";
    }
    std::cout.unsetf(std::ios::fixed);

    // 与批量解对比
    const auto batchGraph = buildPoseGraph(data, odomNoise, loopNoise, true);
    const Values batch =
        LevenbergMarquardtOptimizer(batchGraph, identityInitial()).optimize();
    double max_diff = 0.0;
    for (size_t i = 0; i < kNumPoses; ++i) {
      const Vector3 d = Pose2::Logmap(
          after.at<Pose2>(X(i)).between(batch.at<Pose2>(X(i))));
      max_diff = std::max(max_diff, d.cwiseAbs().maxCoeff());
    }
    std::cout << "\n  iSAM2 结果与批量 LM 结果的最大差异 = " << std::scientific
              << max_diff << std::defaultfloat
              << "  （应为数值噪声量级）\n";
  }

  return 0;
}
