#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <iostream>
#include <memory>
#include <string>

namespace gtsam_learning {

/**
 * @brief 二维"位置观测"因子（类似 GPS / 全局定位观测）
 *
 * 模型：e = T.translation() - z,  e ∈ R^2
 *   - T：机器人位姿 (Pose2)
 *   - z：观测到的世界坐标位置 (Point2)
 *
 * 与 RangeFactor 的区别：
 *   - 残差是 2 维而不是 1 维；可以直接用 noiseModel::Diagonal 表达
 *     "x 方向很准、y 方向很糙" 这样的各向异性噪声。
 *   - 该因子只约束平移，不约束朝向；单独使用会让 Hessian 在 θ 方向奇异，
 *     所以通常要配合别的因子或一个只约束 θ 的弱先验。
 */
class GpsPointFactor
    : public gtsam::NoiseModelFactorT<gtsam::Vector2, gtsam::Pose2> {
 public:
  using Base = gtsam::NoiseModelFactorT<gtsam::Vector2, gtsam::Pose2>;
  using Base::evaluateError;

  /// 参数顺序刻意与官方因子保持一致：(keys..., measured, noiseModel)
  ///   gtsam::GPSFactor(Key key, const Point3& gpsIn,
  ///                    const SharedNoiseModel& model)   // navigation/GPSFactor.h:69
  /// 而基类的构造函数顺序正好相反（noiseModel 在最前），别混了。
  GpsPointFactor(gtsam::Key poseKey, const gtsam::Point2& measured,
                 const gtsam::SharedNoiseModel& noiseModel)
      : Base(noiseModel, poseKey), measured_(measured) {}

  gtsam::Vector2 evaluateError(const gtsam::Pose2& T,
                               gtsam::Matrix* H_T = nullptr) const override {
    gtsam::Matrix23 D_T;
    // Pose2::translation 给出投影矩阵 [I2 | 0]，正好是 ∂t/∂(vx,vy,ω)
    const gtsam::Point2 t = T.translation(H_T ? &D_T : nullptr);
    if (H_T) *H_T = D_T;  // 2x3
    return t - measured_;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new GpsPointFactor(*this)));
  }

  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    Base::print(s + "GpsPointFactor", keyFormatter);
    std::cout << "  measured position = " << measured_.transpose() << std::endl;
  }

  const gtsam::Point2& measured() const { return measured_; }

 private:
  gtsam::Point2 measured_;
};

}  // namespace gtsam_learning
