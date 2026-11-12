/**
 * Lesson 0：模板课 —— 写一个自定义因子最少需要哪些代码
 *
 * 这一课不讲新理论，只给你一份可以**直接抄走**的骨架：
 *   1. 一个自定义因子类（4 件事：基类 / 构造函数 / evaluateError / clone）
 *   2. 手写雅可比的数值自检（写完立刻验证，别等到优化不收敛才怀疑）
 *   3. 搭图 → 给初值 → 求解 → 读边缘协方差（每节课都是这 4 步）
 *   4. 为什么每个子图至少需要一个先验：去掉先验后 Hessian 秩亏
 *      （本例 nullity = 2，对应整张图在 x/y 上的全局平移）
 *
 * 例子刻意选得比 Lesson 1 更简单：观测是「两点之间的平面位移」
 *   e = (p_to - p_from) - z      （2 维残差，雅可比是常数 ±I）
 * 没有流形、没有三角函数，只留纯粹的骨架，方便你套到自己的问题上。
 *
 * 运行：xmake run lesson0
 *
 * TODO 练习：
 *   1. 把残差换成「只有距离」的 1 维因子（e = ||p_to - p_from|| - z，
 *      雅可比 = (Δ/||Δ||)^T 乘 ±I），再跑第 5 节：零特征值会变成几个？
 *      （提示：距离观测丢掉了方向信息，秩会更低。）
 *   2. 把 sigma 从 0.05 改到 0.5，看第 3 节的误差和第 4 节的 sigma
 *      是否同步放大（sqrt(10) 倍）。
 *   3. 给 P1 也加一个先验（σ=0.05），观察 P1/P2 的边缘方差如何变小。
 *   4. 自己写一个新因子：z = p_to - p_from 的「旋转 90 度」版本，
 *      即 z = R(pi/2) * (p_to - p_from)，雅可比不再是 ±I。
 */

#include "lesson_utils.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/slam/PriorFactor.h>

#include <Eigen/Eigenvalues>

#include <array>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <utility>
#include <vector>

using namespace gtsam;
using symbol_shorthand::P;

// ---------------------------------------------------------------------------
// 第 1 件事：因子类本身
// ---------------------------------------------------------------------------
namespace gtsam_learning {

/**
 * @brief 两点之间的平面位移因子：z ≈ p_to - p_from
 *
 *   e      = (p_to - p_from) - z                      2x1
 *   ∂e/∂p_from = -I                                   2x2，常数
 *   ∂e/∂p_to   = +I                                   2x2，常数
 *
 * 四个必备件：
 *   ① 继承 NoiseModelFactorT<OutputVec, Args...>
 *      OutputVec 是残差类型（这里是 Vector2），Args 是每个变量的类型。
 *   ② 构造函数里把 (noiseModel, key1, key2) 交给 Base —— 顺序别搞反。
 *   ③ 实现 evaluateError(...)：返回「未白化」残差，并按需填 H（同样是未白化）。
 *      Σ^{-1/2} 由 GTSAM 在 NoiseModelFactor::linearize() 里统一乘上去。
 *   ④ 实现 clone()：iSAM2 / 重排序 / 序列化会用到，批量优化不强制但应写。
 */
class OffsetFactor
    : public gtsam::NoiseModelFactorT<gtsam::Vector2, gtsam::Point2,
                                      gtsam::Point2> {
public:
  using Base =
      gtsam::NoiseModelFactorT<gtsam::Vector2, gtsam::Point2, gtsam::Point2>;
  /// 一定要有：否则派生类里看不到基类的 2 参 / 3 参 evaluateError 重载
  using Base::evaluateError;

  /// 参数顺序照抄官方因子：(keys..., measured, noiseModel)
  ///   gtsam::BetweenFactor(Key key1, Key key2, const VALUE& measured,
  ///                        const SharedNoiseModel& model = nullptr)
  /// 注意基类的顺序相反（noiseModel 在最前），这里只是"看起来不一样"。
  OffsetFactor(gtsam::Key fromKey, gtsam::Key toKey,
               const gtsam::Vector2 &measured,
               const gtsam::SharedNoiseModel &noiseModel)
      : Base(noiseModel, fromKey, toKey), measured_(measured) {}

  gtsam::Vector2 evaluateError(const gtsam::Point2 &p_from,
                               const gtsam::Point2 &p_to,
                               gtsam::Matrix *H_from = nullptr,
                               gtsam::Matrix *H_to = nullptr) const override {
    // 注意：evaluateError 会被调用非常多次（每次线性化），
    // 这里绝对不能加 std::cout 调试打印，否则会把 lesson 的输出刷爆。
    if (H_from)
      // 用逗号初始化而不是 -Matrix2::Identity()：后者会打印出 "-0"
      *H_from = (gtsam::Matrix2() << -1.0, 0.0, 0.0, -1.0).finished();
    if (H_to)
      *H_to = gtsam::Matrix2::Identity();
    return (p_to - p_from) - measured_;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new OffsetFactor(*this)));
  }

  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    Base::print(s + "OffsetFactor", keyFormatter);
    std::cout << "  measured offset = " << measured_.transpose().eval()
              << std::endl;
  }

  const gtsam::Vector2 &measured() const { return measured_; }

private:
  gtsam::Vector2 measured_;
};

} // namespace gtsam_learning

using gtsam_learning::OffsetFactor;

namespace {

constexpr size_t kNumPoints = 3;
constexpr double kOffsetSigma = 0.05; ///< 位移观测噪声（各向同性, m）
constexpr double kPriorSigma = 0.02; ///< P0 的先验噪声（m）

/// 场景：3 个平面点 + 3 条位移观测，其中 (0,2) 是冗余边（可检验一致性）
struct Scenario {
  std::array<Point2, kNumPoints> truth{Point2(0.0, 0.0), Point2(1.0, 0.6),
                                       Point2(1.8, 1.6)};
  /// 边 (i, j) 的含义：z ≈ p_j - p_i
  std::vector<std::pair<size_t, size_t>> edges{{0, 1}, {1, 2}, {0, 2}};
  std::vector<Vector2> offsets; ///< 与 edges 一一对应
};

Scenario makeScenario(double sigma = kOffsetSigma, unsigned seed = 42) {
  Scenario s;
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, sigma);
  for (const auto &[i, j] : s.edges) {
    Vector2 z = s.truth[j] - s.truth[i];
    z += Vector2(noise(rng), noise(rng)); // 加测量噪声
    s.offsets.push_back(z);
  }
  return s;
}

/// 搭图：先验（可选）+ 每条边的位移因子
NonlinearFactorGraph buildGraph(const Scenario &s,
                                const SharedNoiseModel &offsetNoise,
                                const SharedNoiseModel &priorNoise,
                                bool withPrior = true) {
  NonlinearFactorGraph graph;
  if (withPrior) {
    graph.emplace_shared<PriorFactor<Point2>>(P(0), s.truth[0], priorNoise);
  }
  for (size_t k = 0; k < s.edges.size(); ++k) {
    const auto [i, j] = s.edges[k];
    graph.emplace_shared<OffsetFactor>(P(i), P(j), s.offsets[k], offsetNoise);
  }
  return graph;
}

/// 初值：真值整体平移 shift（故意给一个不准的初值，看优化器怎么拉回来）
Values makeInitial(const Scenario &s, const Point2 &shift) {
  Values initial;
  for (size_t i = 0; i < kNumPoints; ++i) {
    initial.insert(P(i), Point2(s.truth[i] + shift));
  }
  return initial;
}

/// 打印逐点误差表
void printEstimates(const Values &result, const Scenario &s,
                    const std::string &title) {
  // 用 snprintf 拼行，保证列对齐（operator<< 的 setw 在混合类型时容易错位）
  const auto snap = [](double v) { return std::abs(v) < 5e-4 ? 0.0 : v; };
  std::cout << "  " << title << "\n"
            << "   点        真值                估计              误差(m)\n";
  for (size_t i = 0; i < kNumPoints; ++i) {
    const Point2 est = result.at<Point2>(P(i));
    const Point2 truth = s.truth[i];
    char line[160];
    std::snprintf(line, sizeof(line),
                  "   P%zu   (%7.3f,%7.3f)   (%7.3f,%7.3f)   %7.3f\n", i,
                  snap(truth.x()), snap(truth.y()), snap(est.x()),
                  snap(est.y()), (est - truth).norm());
    std::cout << line;
  }
}

} // namespace

int main() {
  const Scenario s = makeScenario();
  const auto offsetNoise = noiseModel::Isotropic::Sigma(2, kOffsetSigma);
  const auto priorNoise = noiseModel::Isotropic::Sigma(2, kPriorSigma);

  // -------------------------------------------------------------------------
  lesson::section("1. 一个自定义因子：残差、雅可比、白化");
  {
    const OffsetFactor factor(P(0), P(1), s.offsets[0], offsetNoise);
    const Point2 p_from(0.0, 0.0);
    const Point2 p_to(1.0, 2.0);

    Matrix H_from, H_to;
    const Vector2 e = factor.evaluateError(p_from, p_to, &H_from, &H_to);

    std::cout
        << "  z = " << s.offsets[0].transpose().eval() << "（带噪声的观测量）\n"
        << "  在 p_from=(0,0), p_to=(1,2) 处求值：\n"
        << "    e      = (p_to - p_from) - z = " << e.transpose().eval()
        << "\n    H_from = -I =\n"
        << H_from << "\n    H_to   = +I =\n"
        << H_to << "\n"
        << "  未白化残差 e 直接进优化器是错的，GTSAM 会替你乘上 Σ^{-1/2}：\n"
        << "    whiten(e) = " << offsetNoise->whiten(e).transpose().eval()
        << "  (sigma=" << kOffsetSigma << " → 除以 sigma)\n"
        << "    error    = 0.5*||whiten(e)||^2 = "
        << 0.5 * offsetNoise->whiten(e).squaredNorm() << "\n";
  }

  // -------------------------------------------------------------------------
  lesson::section("2. 自检：手写雅可比 vs 中心差分（写完因子第一件事）");
  {
    const auto graph = buildGraph(s, offsetNoise, priorNoise);
    size_t edge = 0; // 图里第 0 个是先验，边要单独计数，不能拿图索引去查 edges
    for (size_t k = 0; k < graph.size(); ++k) {
      const auto factor = std::dynamic_pointer_cast<OffsetFactor>(graph[k]);
      if (!factor)
        continue; // 先验因子是内置的，不需要自检
      const auto [i, j] = s.edges[edge++];
      // 线性化点故意取远离真值的初值，避免误差恰好为 0 掩盖 bug
      Values probe;
      probe.insert(P(i), Point2(s.truth[i] + Point2(0.3, -0.2)));
      probe.insert(P(j), Point2(s.truth[j] + Point2(-0.1, 0.4)));
      lesson::printCheck("OffsetFactor(P" + std::to_string(i) + ",P" +
                             std::to_string(j) + ")",
                         lesson::checkJacobian(*factor, probe));
    }
    std::cout << "  → 全 OK 才说明雅可比没写错；FAIL 就先别急着调优化器\n"
                 "    的参数（Lesson 1 第 2 节演示了写错雅可比会怎样）。\n";
  }

  // -------------------------------------------------------------------------
  lesson::section("3. 搭图 → 给初值 → 求解");
  {
    const auto graph = buildGraph(s, offsetNoise, priorNoise);
    const Values initial = makeInitial(s, Point2(0.35, -0.25));

    std::cout << "  因子：" << graph.size()
              << " 个（1 个 P0 先验 + 3 个位移观测）\n"
              << "  初值：真值整体加 (0.35, -0.25) 的偏移\n";

    // 冗余边的「一致性检查」：不改图、不优化，先看测量之间是否自洽
    const Vector2 inconsistency = s.offsets[2] - (s.offsets[0] + s.offsets[1]);
    std::cout << "  冗余边一致性：z02 - (z01 + z12) = "
              << inconsistency.transpose().eval()
              << "  (norm=" << inconsistency.norm()
              << " m，量级应与 sigma 相当)\n";

    lesson::Timer timer;
    const Values result =
        LevenbergMarquardtOptimizer(graph, initial).optimize();
    const double ms = timer.ms();

    std::cout << "\n  LM 用时 " << std::fixed << std::setprecision(3) << ms
              << " ms,  error: " << graph.error(initial) << " -> "
              << graph.error(result) << "\n";
    std::cout.unsetf(std::ios::fixed);
    printEstimates(result, s, "求解结果：");
  }

  // -------------------------------------------------------------------------
  lesson::section("4. 边缘协方差：谁的观测更多，谁就更确定");
  {
    const auto graph = buildGraph(s, offsetNoise, priorNoise);
    const Values result =
        LevenbergMarquardtOptimizer(graph, makeInitial(s, Point2(0.0, 0.0)))
            .optimize();
    const Marginals marginals(graph, result);

    std::cout << "   点      sigma_x      sigma_y     sqrt(det)   信息源\n";
    const char *why[kNumPoints] = {"先验 + 2 条边", "2 条边",
                                   "2 条边（含冗余边）"};
    for (size_t i = 0; i < kNumPoints; ++i) {
      const Matrix2 cov = marginals.marginalCovariance(P(i));
      char line[160];
      std::snprintf(line, sizeof(line), "   P%zu%12.4f%13.4f%13.4f   %s\n", i,
                    std::sqrt(cov(0, 0)), std::sqrt(cov(1, 1)),
                    std::sqrt(cov.determinant()), why[i]);
      std::cout << line;
    }
    std::cout
        << "  → sigma 由「有效信息量」决定：P0 有先验 + 2 条边，最确定；\n"
           "    P1/P2 只靠边推，方差更大。边缘协方差 = (H^{-1}) 的对角块，\n"
           "    和 Lesson 4 第 3 节（GPS 内点 vs 跳变帧）是同一件事。\n";
  }

  // -------------------------------------------------------------------------
  lesson::section("5. 去掉先验：整张图秩亏（gauge freedom）");
  {
    const auto graph =
        buildGraph(s, offsetNoise, priorNoise, /*withPrior=*/false);
    const Values initial = makeInitial(s, Point2(0.0, 0.0));

    // 线性化后看整张图的 Hessian：零特征值个数 = 不可观方向个数
    const auto linear = graph.linearize(initial);
    const Matrix H = linear->hessian().first;
    Eigen::SelfAdjointEigenSolver<Matrix> eig(H);
    const double max_eig = eig.eigenvalues().maxCoeff();
    size_t nullity = 0;
    for (int i = 0; i < eig.eigenvalues().size(); ++i) {
      if (eig.eigenvalues()(i) < 1e-9 * max_eig)
        ++nullity;
    }
    std::cout << "  Hessian 维度 = " << H.rows() << "x" << H.cols()
              << "，零特征值个数 = " << nullity
              << "\n  特征值 = " << eig.eigenvalues().transpose() << "\n"
              << "  → 位移观测只约束「相对位置」：整张图平移不改变任何残差，\n"
                 "    所以恰好有 2 个不可观方向（x、y 的全局平移）。\n";

    // GaussNewton 遇到奇异 Hessian 会直接抛异常，LM 靠阻尼把它「糊」过去
    try {
      GaussNewtonOptimizer(graph, initial).optimize();
      std::cout << "  （GaussNewton 居然没抛异常？）\n";
    } catch (const IndeterminantLinearSystemException &e) {
      std::cout << "\n  GaussNewton 抛 IndeterminantLinearSystemException，"
                   "第一个出问题的变量 = "
                << DefaultKeyFormatter(e.nearbyVariable()) << "\n"
                << "  → 秩亏时线性方程没有唯一解（GN 不做阻尼，直接失败）；\n"
                   "    LM 加了阻尼所以能「解」，但解出来的只是某个满足相对\n"
                   "    约束的形状。\n";
    }

    // 两个不同的初值 → 不同的绝对位置，但形状（相对位移）完全一样
    const Values fromA = LevenbergMarquardtOptimizer(graph, initial).optimize();
    const Values fromB =
        LevenbergMarquardtOptimizer(graph, makeInitial(s, Point2(5.0, -3.0)))
            .optimize();

    const Vector2 dA = fromA.at<Point2>(P(1)) - fromA.at<Point2>(P(0));
    const Vector2 dB = fromB.at<Point2>(P(1)) - fromB.at<Point2>(P(0));

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n  初值 P0=( 0.000, 0.000) → 解 P0=("
              << fromA.at<Point2>(P(0)).x() << ", "
              << fromA.at<Point2>(P(0)).y() << "),  P1-P0=(" << dA.x() << ", "
              << dA.y() << ")\n"
              << "  初值 P0=( 5.000,-3.000) → 解 P0=("
              << fromB.at<Point2>(P(0)).x() << ", "
              << fromB.at<Point2>(P(0)).y() << "),  P1-P0=(" << dB.x() << ", "
              << dB.y() << ")\n";
    std::cout.unsetf(std::ios::fixed);
    std::cout
        << std::setprecision(6) << "  两条解的 error = " << graph.error(fromA)
        << " vs " << graph.error(fromB)
        << "（相同），P1-P0 之差 = " << (dA - dB).norm() << " m\n"
        << "  → 绝对位置由初值决定（不可观），相对形状由观测决定（可观）。\n"
           "    工程上永远要问：我的图里，谁负责钉住这个 gauge？\n"
           "    （回环 SLAM 里是「第一个位姿的先验」，VIO 里是重力方向。）\n";
  }

  return 0;
}
