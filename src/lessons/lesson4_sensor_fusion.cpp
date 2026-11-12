/**
 * Lesson 4：里程计 + GPS 融合 —— 为什么"融合"比"二选一"好
 *
 * 学习目标：
 *   1. 里程计（相对约束，稠密、会漂移）与 GPS（绝对约束，稀疏、会跳变）如何互补
 *   2. 用同一套误差指标（位置 RMSE / 朝向 RMSE）横向比较：
 *      死推、只用 GPS、因子图融合、因子图 + 鲁棒核
 *   3. 读边缘协方差：有 GPS 的位姿、没有 GPS 的位姿，不确定度差多少
 *
 * 运行：xmake run lesson4
 *
 * TODO 练习：
 *   1. 把 GPS 噪声从 (0.3, 0.3) 改成各向异性 (0.1, 0.8)（用 Diagonal），
 *      观察 x/y 两个方向的 RMSE 与协方差椭圆如何变化。
 *   2. 把 GPS 的频率从每 3 帧降到每 6 帧，看融合结果的 RMSE 如何变化——
 *      这决定了"回环/绝对观测要多久来一次"。
 */

#include "factors/gps_point_factor.hpp"
#include "lesson_utils.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <vector>

using namespace gtsam;
using gtsam_learning::GpsPointFactor;

namespace {

using symbol_shorthand::X;

constexpr size_t kN = 20;          ///< 位姿数
constexpr size_t kGpsStride = 3;   ///< 每 3 帧来一次 GPS

struct Data {
  std::vector<Pose2> truth;
  std::vector<Pose2> odometry;      ///< size = kN - 1
  std::map<size_t, Point2> gps;     ///< 位姿下标 -> 观测（含 2 个外点）
};

Data makeData() {
  Data d;
  for (size_t i = 0; i < kN; ++i) {
    const double s = 0.3 * static_cast<double>(i);
    // 半径 5 m 的圆，朝向沿切线
    d.truth.emplace_back(5.0 * std::cos(s), 5.0 * std::sin(s), s + M_PI_2);
  }

  std::mt19937 rng(7);
  std::normal_distribution<double> odom_noise(0.0, 0.15);
  std::normal_distribution<double> gps_noise(0.0, 0.25);

  for (size_t i = 0; i + 1 < kN; ++i) {
    d.odometry.push_back(
        d.truth[i].between(d.truth[i + 1]) *
        Pose2(odom_noise(rng), odom_noise(rng), 0.3 * odom_noise(rng)));
  }
  for (size_t i = 0; i < kN; i += kGpsStride) {
    Point2 z = d.truth[i].translation() +
               Point2(gps_noise(rng), gps_noise(rng));
    // 两个 GPS 跳变（城市峡谷 / 多径）
    if (i == 6 || i == 15) z += Point2(4.0, -3.0);
    d.gps[i] = z;
  }
  return d;
}

NonlinearFactorGraph buildGraph(const Data& d, const SharedNoiseModel& odomNoise,
                                const SharedNoiseModel& gpsNoise,
                                bool with_gps, bool robust_gps) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose2>>(
      X(0), d.truth[0], noiseModel::Isotropic::Sigma(3, 1e-3));

  for (size_t i = 0; i + 1 < kN; ++i) {
    graph.emplace_shared<BetweenFactor<Pose2>>(X(i), X(i + 1), d.odometry[i],
                                               odomNoise);
  }
  if (with_gps) {
    const SharedNoiseModel model =
        robust_gps
            ? noiseModel::Robust::Create(
                  noiseModel::mEstimator::Cauchy::Create(1.0), gpsNoise)
            : gpsNoise;
    for (const auto& [idx, z] : d.gps) {
      graph.emplace_shared<GpsPointFactor>(X(idx), z, model);
    }
  }
  return graph;
}

/// 死推：从 X0 出发，把里程计一路乘下去
std::vector<Pose2> deadReckoning(const Data& d) {
  std::vector<Pose2> est(kN);
  est[0] = d.truth[0];
  for (size_t i = 0; i + 1 < kN; ++i) est[i + 1] = est[i] * d.odometry[i];
  return est;
}

/// 死推结果作为优化初值（工程上最常用的初值策略）
Values valuesAsInitial(const Data& d) {
  const auto est = deadReckoning(d);
  Values initial;
  for (size_t i = 0; i < kN; ++i) initial.insert(X(i), est[i]);
  return initial;
}

std::vector<Pose2> valuesToVector(const Values& v) {
  std::vector<Pose2> out;
  for (size_t i = 0; i < kN; ++i) out.push_back(v.at<Pose2>(X(i)));
  return out;
}

/// 位置 RMSE / 朝向 RMSE，只在 valid[i] 为真的位姿上统计
std::pair<double, double> rmse(const std::vector<Pose2>& est, const Data& d,
                               const std::vector<bool>& valid) {
  double sum_xy = 0.0;
  double sum_th = 0.0;
  size_t n = 0;
  for (size_t i = 0; i < kN; ++i) {
    if (!valid[i]) continue;
    const Vector3 e = lesson::poseError(est[i], d.truth[i]);
    sum_xy += e.head<2>().squaredNorm();
    sum_th += e.z() * e.z();
    ++n;
  }
  return {std::sqrt(sum_xy / n), std::sqrt(sum_th / n)};
}

}  // namespace

int main() {
  const Data d = makeData();
  const auto odomNoise =
      noiseModel::Diagonal::Sigmas(Vector3(0.15, 0.15, 0.05));
  const auto gpsNoise = noiseModel::Isotropic::Sigma(2, 0.25);
  const std::vector<bool> all_poses(kN, true);

  lesson::section("1. 各信息源单独使用");
  {
    const auto dr = deadReckoning(d);
    const auto [xy, th] = rmse(dr, d, all_poses);
    std::cout << "  里程计死推 (" << kN << " 帧):        xy_RMSE="
              << std::fixed << std::setprecision(3) << xy
              << " m,  theta_RMSE=" << th << " rad\n"
              << "    其中最后一帧 X" << kN - 1 << " 的位置误差 = "
              << lesson::poseError(dr[kN - 1], d.truth[kN - 1])
                     .head<2>()
                     .norm()
              << " m（漂移随里程累积）\n";

    std::vector<Pose2> gps_only(kN);
    std::vector<bool> inlier(kN, false);
    std::vector<bool> all_gps(kN, false);
    size_t n_inlier = 0;
    for (const auto& [idx, z] : d.gps) {
      // GPS 只测位置：朝向这里用真值填，仅用于比较位置精度
      gps_only[idx] = Pose2(z.x(), z.y(), d.truth[idx].theta());
      all_gps[idx] = true;
      if (idx != 6 && idx != 15) {
        inlier[idx] = true;
        ++n_inlier;
      }
    }
    const auto [gxy, gth] = rmse(gps_only, d, inlier);
    const auto [axy, ath] = rmse(gps_only, d, all_gps);
    std::cout << "  只用 GPS（" << n_inlier << " 个内点帧）:  xy_RMSE="
              << std::fixed << std::setprecision(3) << gxy
              << " m；把 2 个跳变帧也算进去: " << axy
              << " m（朝向无观测，其余 " << kN - d.gps.size()
              << " 帧完全不可知）\n";
  }

  lesson::section("2. 融合：不处理外点的话，最小二乘会被 GPS 跳变毁掉");
  {
    struct Case {
      std::string name;
      bool robust;
    };
    const std::vector<Case> cases{{"普通高斯 GPS", false},
                                  {"Cauchy 鲁棒 GPS", true}};

    for (const auto& c : cases) {
      const auto graph = buildGraph(d, odomNoise, gpsNoise, true, c.robust);
      LevenbergMarquardtOptimizer optimizer(graph, valuesAsInitial(d));
      const Values result = optimizer.optimize();
      const auto est = valuesToVector(result);
      const auto [xy, th] = rmse(est, d, all_poses);

      double outlier_xy = 0.0;
      for (size_t idx : {size_t{6}, size_t{15}}) {
        outlier_xy += lesson::poseError(est[idx], d.truth[idx]).head<2>().norm();
      }

      std::cout << "  " << std::left << std::setw(18) << c.name << std::right
                << "xy_RMSE=" << std::fixed << std::setprecision(3) << xy
                << " m,  theta_RMSE=" << std::setprecision(4) << th
                << ",  外点帧平均位置误差=" << std::setprecision(3)
                << 0.5 * outlier_xy << " m,  cost(该模型下)="
                << graph.error(result) << "\n";
    }
    std::cout
        << "\n  → 里程计给「连续性」（每帧都有，但会漂），GPS 给「绝对性」\n"
           "    （稀疏，且有 0.25 m 噪声和偶发跳变）。普通最小二乘对跳变\n"
           "    毫无抵抗力：2 个坏点就能把整条轨迹拖得比裸死推还差；\n"
           "    鲁棒核把坏点降权之后，融合才真正优于单用里程计。\n";
  }

  lesson::section("3. 边缘协方差：有效信息越多越确定");
  {
    const auto graph = buildGraph(d, odomNoise, gpsNoise, true, true);
    const Values result = LevenbergMarquardtOptimizer(graph, valuesAsInitial(d)).optimize();
    const Marginals marginals(graph, result);

    std::cout << "  位姿   有GPS?      sigma_x      sigma_y     sigma_theta\n";
    for (size_t i : {size_t{12}, size_t{13}, size_t{14}, size_t{15}}) {
      const Matrix3 cov = marginals.marginalCovariance(X(i));
      std::cout << "   X" << std::left << std::setw(4) << i << std::setw(8)
                << (d.gps.count(i) ? "yes" : "no") << std::right
                << std::fixed << std::setprecision(4) << std::setw(11)
                << std::sqrt(cov(0, 0)) << std::setw(13)
                << std::sqrt(cov(1, 1)) << std::setw(13)
                << std::sqrt(cov(2, 2)) << "\n";
    }
    std::cout.unsetf(std::ios::fixed);
    std::cout
        << "  → X12 有 GPS 内点，比靠里程计链推出来的 X13/X14 更确定；\n"
           "    X15 虽然也有 GPS，但它正是跳变帧，被鲁棒核降权，\n"
           "    所以协方差反而最大 —— 边缘协方差反映的是「有效信息」，\n"
           "    不是「有没有这个传感器」。\n";
  }

  return 0;
}
