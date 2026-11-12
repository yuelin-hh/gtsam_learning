#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <cmath>
#include <memory>
#include <string>

namespace gtsam_learning {

/**
 * @brief 一维测距因子（Range-only Factor）
 *
 * 模型：e = || T^{-1} * p || - z
 *   - T：机器人位姿 (Pose2，3 个自由度)
 *   - p：世界坐标系下的路标 (Point2，2 个自由度)
 *   - z：测得的距离（标量，1 维残差）
 *
 * 为什么用 NoiseModelFactorT：
 *   - 它是 GTSAM 4.3 推荐的 N 元因子基类，编译期就知道每个变量的类型与维度；
 *   - 只需要实现 evaluateError，GTSAM 负责白化、线性化、稀疏结构等全部工作。
 *
 * 雅可比推导（关键！）：
 *   l      = T.transformTo(p)          // p 在 T 的局部坐标系下的坐标，2x1
 *   range  = ||l||                     // 标量
 *   e      = range - z
 *   ∂e/∂l  = l^T / ||l||               // 1x2
 *   ∂l/∂T  = D_T (2x3)                 // transformTo 给出的、关于 T 切空间的导数
 *   ∂l/∂p  = D_p (2x2)
 *   => H_T = (l^T/||l||) * D_T         // 1x3
 *      H_p = (l^T/||l||) * D_p         // 1x2
 *
 * 注意：这里返回的是"未白化"的残差和雅可比，噪声模型 Σ^{-1/2} 由 GTSAM
 *       在 NoiseModelFactor::linearize() 里统一乘上去。
 */
class RangeFactor
    : public gtsam::NoiseModelFactorT<gtsam::Vector, gtsam::Pose2, gtsam::Point2> {
 public:
  using Base =
      gtsam::NoiseModelFactorT<gtsam::Vector, gtsam::Pose2, gtsam::Point2>;
  /// 便于在派生类里访问 X1 / X2 / evaluateError 重载集合
  using Base::evaluateError;

  /// 参数顺序刻意与官方因子保持一致：(keys..., measured, noiseModel)
  ///   gtsam::RangeFactor(Key key1, Key key2, T measured,
  ///                      const SharedNoiseModel& model)   // sam/RangeFactor.h:49
  /// 而基类的构造函数顺序正好相反（noiseModel 在最前），别混了：
  ///   NoiseModelFactor(const SharedNoiseModel&);
  ///   NoiseModelFactorT(noiseModel, keys...);
  RangeFactor(gtsam::Key poseKey, gtsam::Key pointKey, double measured,
              const gtsam::SharedNoiseModel& noiseModel)
      : Base(noiseModel, poseKey, pointKey), measured_(measured) {}

  gtsam::Vector evaluateError(const gtsam::Pose2& T, const gtsam::Point2& p,
                              gtsam::Matrix* H_T = nullptr,
                              gtsam::Matrix* H_p = nullptr) const override {
    gtsam::Matrix23 D_T;  // ∂l/∂T，关于 Pose2 的 3 维切空间
    gtsam::Matrix22 D_p;  // ∂l/∂p

    const gtsam::Point2 l =
        T.transformTo(p, H_T ? &D_T : nullptr, H_p ? &D_p : nullptr);
    const double range = l.norm();

    gtsam::Vector1 e;
    e << range - measured_;

    if (H_T || H_p) {
      gtsam::Matrix12 J;  // ∂‖l‖/∂l
      if (range > kEpsilon) {
        J << l.x() / range, l.y() / range;
      } else {
        // 退化情形：路标正好落在相机/机器人原点，距离对方向的导数不存在
        J.setZero();
      }
      if (H_T) *H_T = J * D_T;  // 1x3
      if (H_p) *H_p = J * D_p;  // 1x2
    }

    return e;
  }

  /// ISAM2 重新排序 / 序列化时需要；批量优化不强制，但属于好习惯
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new RangeFactor(*this)));
  }

  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    Base::print(s + "RangeFactor", keyFormatter);
    std::cout << "  measured range = " << measured_ << std::endl;
  }

  double measured() const { return measured_; }

 private:
  static constexpr double kEpsilon = 1e-9;
  double measured_;
};

}  // namespace gtsam_learning
