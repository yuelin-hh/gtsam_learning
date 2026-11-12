#pragma once

/**
 * @file lesson_utils.hpp
 * @brief 五节课共用的小工具：雅可比自检、因子误差表、Pose2 误差、计时、章节标题。
 *
 * 全部是 header-only 的 inline 函数，任意 lesson 直接 #include "lesson_utils.hpp" 即可。
 * 依赖只有 GTSAM + 标准库，不依赖任何 lesson 的私有代码。
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace lesson {

/// 雅可比自检结果
struct JacobianCheck {
  bool ok{false};
  double max_abs_error{0.0};   ///< max |J_analytic - J_numeric|
  double relative_error{0.0};  ///< max_abs_error / max|J_numeric|
};

/**
 * @brief 用中心差分数值雅可比校验因子手写的解析雅可比
 *
 * 原理：gtsam::linearizeNumerically 会对因子按 keys() 顺序逐个变量做中心差分，
 *       得到一个"白化后"的 JacobianFactor；factor.linearize(values) 得到的是
 *       解析雅可比线性化结果（同样白化）。两者逐块比较即可。
 *
 * @param factor 待测因子
 * @param values 线性化点（最好选一个误差非零、姿态不平庸的点）
 * @param delta  数值差分步长
 * @param tol    判定通过的最大相对误差
 */
inline JacobianCheck checkJacobian(const gtsam::NoiseModelFactor& factor,
                                  const gtsam::Values& values,
                                  double delta = 1e-6, double tol = 1e-5) {
  const auto actual =
      std::dynamic_pointer_cast<gtsam::JacobianFactor>(factor.linearize(values));
  if (!actual) {
    throw std::runtime_error(
        "checkJacobian: factor.linearize(values) 未返回 JacobianFactor");
  }

  gtsam::JacobianFactor expected =
      gtsam::linearizeNumerically(factor, values, delta);

  JacobianCheck result;
  double scale = 0.0;
  for (gtsam::Key key : factor.keys()) {
    const gtsam::Matrix J_actual = actual->getA(key);
    const gtsam::Matrix J_expected = expected.getA(key);
    if (J_actual.rows() != J_expected.rows() ||
        J_actual.cols() != J_expected.cols()) {
      // 维度都不对（常见于 evaluateError 里把 H 的顺序/形状写错）
      result.ok = false;
      result.max_abs_error = std::numeric_limits<double>::infinity();
      result.relative_error = std::numeric_limits<double>::infinity();
      return result;
    }
    result.max_abs_error =
        std::max(result.max_abs_error,
                 (J_actual - J_expected).cwiseAbs().maxCoeff());
    scale = std::max(scale, J_expected.cwiseAbs().maxCoeff());
  }
  result.relative_error =
      result.max_abs_error / std::max(scale, 1e-12);
  result.ok = result.relative_error <= tol;
  return result;
}

/// 打印一条雅可比自检结论（[ OK ] / [FAIL] + 最大绝对误差与相对误差）
inline void printCheck(const std::string& name, const JacobianCheck& r) {
  std::cout << (r.ok ? "[ OK ] " : "[FAIL] ") << std::left << std::setw(34)
            << name << std::scientific << std::setprecision(2)
            << " max|J_ana-J_num| = " << r.max_abs_error
            << "  rel = " << r.relative_error << std::defaultfloat
            << std::setprecision(6) << std::endl;
}

/**
 * @brief 打印因子图里每个因子的（白化）误差与总和
 *
 * 用途：定位外点/坏因子。注意 error() 已经包含噪声模型的权重，
 *       所以「残差大」与「error 大」不是一回事 —— 只有高权重因子的
 *       大残差才会真的把解拖走（见 Lesson 1 第 5 节）。
 *
 * @param graph  非线性因子图
 * @param values 要评估的状态（通常是优化结果）
 * @param title  打印标题
 */
inline void printFactorErrors(const gtsam::NonlinearFactorGraph& graph,
                              const gtsam::Values& values,
                              const std::string& title = "factor errors") {
  std::cout << "--- " << title << " ---" << std::endl;
  double total = 0.0;
  for (size_t i = 0; i < graph.size(); ++i) {
    const auto& f = graph[i];
    if (!f) continue;
    const double e = f->error(values);
    total += e;
    std::cout << "  #" << std::setw(2) << std::right << i << "  keys=[";
    for (size_t j = 0; j < f->size(); ++j) {
      std::cout << gtsam::DefaultKeyFormatter(f->keys()[j]);
      if (j + 1 < f->size()) std::cout << ", ";
    }
    std::cout << "]  error=" << std::fixed << std::setprecision(6) << e
              << std::endl;
  }
  std::cout << "  total = " << std::fixed << std::setprecision(6) << total
            << std::endl;
  std::cout.unsetf(std::ios::fixed);
}

/**
 * @brief Pose2 相对真值的切空间误差 [vx, vy, ω]
 *
 * 用 Logmap(between()) 而不是直接相减，因为 θ 有 ±π 回绕、且平移误差
 * 必须在「以估计位姿为原点」的局部坐标系里度量（Lesson 3 第 1 节）。
 */
inline gtsam::Vector3 poseError(const gtsam::Pose2& estimate,
                               const gtsam::Pose2& truth) {
  return gtsam::Pose2::Logmap(estimate.between(truth));
}

/// 简单的毫秒计时器：构造即开始计时，ms() 返回已过去的毫秒数
class Timer {
 public:
  Timer() : start_(std::chrono::steady_clock::now()) {}
  /// 从构造到现在的毫秒数（可反复调用）
  double ms() const {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - start_)
        .count();
  }

 private:
  std::chrono::steady_clock::time_point start_;
};

/// 章节标题（每节课用它分节，输出即讲义目录）
inline void section(const std::string& title) {
  std::cout << "\n================ " << title << " ================"
            << std::endl;
}

}  // namespace lesson
