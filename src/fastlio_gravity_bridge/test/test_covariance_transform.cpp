// 协方差测试：源端状态顺序、右扰动到固定轴的转换、完整旋转、对称性、半正定性，
// 以及姿态误差映射的有限差分检查。
#include <cmath>
#include <limits>
#include <random>

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include "fastlio_gravity_bridge/gravity_alignment.h"

namespace fgb = fastlio_gravity_bridge;

namespace
{

constexpr double kGravity = 9.81;
constexpr double kPitchDeg = 20.12;

Eigen::Matrix3d pitchRotation(double pitch_deg)
{
  return Eigen::AngleAxisd(pitch_deg * M_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

/// 构造一个含非零交叉项、各轴方差不同的对称正定矩阵。
Eigen::Matrix<double, 6, 6> makePositiveDefinite()
{
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
  // 位置（m^2）与姿态（rad^2）量级不同，交叉项非零。
  covariance.diagonal() << 1.0e-2, 4.0e-2, 9.0e-2, 1.0e-5, 4.0e-5, 9.0e-5;
  covariance(0, 1) = covariance(1, 0) = 3.0e-3;
  covariance(0, 3) = covariance(3, 0) = 2.0e-4;
  covariance(1, 4) = covariance(4, 1) = -5.0e-4;
  covariance(2, 5) = covariance(5, 2) = 1.0e-4;
  covariance(3, 4) = covariance(4, 3) = 1.0e-6;
  return covariance;
}

std::mt19937 makeGenerator()
{
  return std::mt19937(20260918u);
}

/// 把 6 维零均值高斯样本写进 out（按 Σ = L Lᵀ 的 Cholesky 因子缩放）。
Eigen::Matrix<double, 6, 1> samplePerturbation(const Eigen::Matrix<double, 6, 6> &sqrt_cov,
                                               std::mt19937 &generator)
{
  Eigen::Matrix<double, 6, 1> z;
  std::normal_distribution<double> normal(0.0, 1.0);
  for (int i = 0; i < 6; ++i) z(i) = normal(generator);
  return sqrt_cov * z;
}

bool isSymmetric(const Eigen::Matrix<double, 6, 6> &m, double tol)
{
  return (m - m.transpose()).cwiseAbs().maxCoeff() < tol;
}

bool isPositiveSemiDefinite(const Eigen::Matrix<double, 6, 6> &m, double tol)
{
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(m);
  return solver.eigenvalues().minCoeff() >= -tol;
}

/// 用消息里的四元数构造固定轴（world 侧）误差：Log(q' ⊗ q⁻¹)。
Eigen::Vector3d fixedAxisRotationError(const geometry_msgs::Quaternion &q_perturbed,
                                       const geometry_msgs::Quaternion &q_nominal)
{
  const Eigen::Quaterniond qp(q_perturbed.w, q_perturbed.x, q_perturbed.y, q_perturbed.z);
  const Eigen::Quaterniond qn(q_nominal.w, q_nominal.x, q_nominal.y, q_nominal.z);
  Eigen::Quaterniond delta = qp * qn.conjugate();
  if (delta.w() < 0.0) delta.coeffs() *= -1.0;  // 取短弧
  delta.normalize();
  const Eigen::AngleAxisd aa(delta);
  return aa.axis() * aa.angle();
}

}  // namespace

/// 源端状态顺序为 [pos, rot]，旋转块按右扰动转换：Σ = A P6 Aᵀ，A = diag(I3, R)。
TEST(PoseCovarianceTransform, SourceStateOrderAndRightPerturbation)
{
  const Eigen::Matrix<double, 6, 6> p6 = makePositiveDefinite();
  const Eigen::Matrix3d R = pitchRotation(kPitchDeg);

  const Eigen::Matrix<double, 6, 6> sigma = fgb::ikfomPoseCovarianceToRosFrame(p6, R);

  // 位置块原样保留（说明源端前三项是位置）。
  EXPECT_LT((sigma.block<3, 3>(0, 0) - p6.block<3, 3>(0, 0)).cwiseAbs().maxCoeff(), 1e-18);
  // 姿态块按 R 旋转（右扰动 -> 固定轴）。
  EXPECT_LT((sigma.block<3, 3>(3, 3) - R * p6.block<3, 3>(3, 3) * R.transpose())
                .cwiseAbs().maxCoeff(), 1e-18);
  // 交叉项按各自方向旋转，完整保留。
  EXPECT_LT((sigma.block<3, 3>(0, 3) - p6.block<3, 3>(0, 3) * R.transpose()).cwiseAbs().maxCoeff(),
            1e-18);
  EXPECT_LT((sigma.block<3, 3>(3, 0) - R * p6.block<3, 3>(3, 0)).cwiseAbs().maxCoeff(), 1e-18);

  // 姿态块确实被旋转过，不是原样照抄。
  EXPECT_GT((sigma.block<3, 3>(3, 3) - p6.block<3, 3>(3, 3)).cwiseAbs().maxCoeff(), 1e-8);

  // 与"位置/姿态块互换"的错误实现明显不同。
  Eigen::Matrix<double, 6, 6> swapped;
  swapped << p6.block<3, 3>(3, 3), p6.block<3, 3>(3, 0), p6.block<3, 3>(0, 3),
      p6.block<3, 3>(0, 0);
  EXPECT_GT((sigma - swapped).cwiseAbs().maxCoeff(), 1e-6);
}

/// 各轴方差不同：位置块与姿态块的迹在旋转下保持不变。
TEST(PoseCovarianceTransform, DifferentAxisVariancesArePreservedOnDiagonal)
{
  Eigen::Matrix<double, 6, 6> p6 = Eigen::Matrix<double, 6, 6>::Zero();
  p6.diagonal() << 0.11, 0.22, 0.33, 1.1e-4, 2.2e-4, 3.3e-4;
  const Eigen::Matrix3d R = pitchRotation(kPitchDeg);

  const Eigen::Matrix<double, 6, 6> sigma = fgb::ikfomPoseCovarianceToRosFrame(p6, R);

  EXPECT_NEAR((sigma.block<3, 3>(0, 0)).trace(), (p6.block<3, 3>(0, 0)).trace(), 1e-15);
  EXPECT_NEAR((sigma.block<3, 3>(3, 3)).trace(), (p6.block<3, 3>(3, 3)).trace(), 1e-15);
  // 纯对角源的姿态块在 R 下不再对角，但主对角线元素仍为正。
  for (int i = 3; i < 6; ++i) EXPECT_GT(sigma(i, i), 0.0);
}

/// 对称性与半正定性：正定源矩阵变换后仍是正定矩阵。
TEST(PoseCovarianceTransform, SymmetryAndPositiveDefiniteness)
{
  Eigen::Matrix<double, 6, 6> p6 = makePositiveDefinite();
  // 叠加一个随机正定矩阵，确保严格正定。
  std::mt19937 generator = makeGenerator();
  std::normal_distribution<double> normal(0.0, 1.0);
  Eigen::Matrix<double, 6, 6> L;
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j) L(i, j) = normal(generator);
  p6 += 1e-3 * (L * L.transpose());

  const Eigen::Matrix3d R = pitchRotation(kPitchDeg);
  const Eigen::Matrix<double, 6, 6> sigma = fgb::ikfomPoseCovarianceToRosFrame(p6, R);

  EXPECT_TRUE(isSymmetric(sigma, 1e-18));
  EXPECT_TRUE(isPositiveSemiDefinite(sigma, 1e-15));
  EXPECT_GT((sigma.block<3, 3>(3, 3)).determinant(), 0.0);
}

/// 桥接节点的位姿/协方差变换：位置、姿态左乘，协方差按 J = diag(R_a, R_a) 传播。
TEST(PoseCovarianceTransform, BridgeTransformMatchesJacobian)
{
  const Eigen::Vector3d gravity_camera =
      pitchRotation(kPitchDeg).transpose() * Eigen::Vector3d(0.0, 0.0, -kGravity);
  fgb::AlignmentTransform transform;
  ASSERT_TRUE(fgb::computeAlignmentTransform(gravity_camera, transform));

  geometry_msgs::PoseWithCovariance in;
  in.pose.position.x = 1.0;
  in.pose.position.y = -2.0;
  in.pose.position.z = 3.0;
  const Eigen::Quaterniond q_cb(Eigen::AngleAxisd(0.4, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()));
  in.pose.orientation.w = q_cb.w();
  in.pose.orientation.x = q_cb.x();
  in.pose.orientation.y = q_cb.y();
  in.pose.orientation.z = q_cb.z();
  const Eigen::Matrix<double, 6, 6> cov_c = makePositiveDefinite();
  fgb::covarianceToMsg(cov_c, in.covariance.elems);

  geometry_msgs::PoseWithCovariance out;
  ASSERT_TRUE(fgb::transformPoseWithCovariance(transform, in, out));

  // 位置左乘 R_a，姿态左乘 q_a。
  const Eigen::Vector3d p_w = transform.R * Eigen::Vector3d(1.0, -2.0, 3.0);
  EXPECT_NEAR(out.pose.position.x, p_w.x(), 1e-12);
  EXPECT_NEAR(out.pose.position.y, p_w.y(), 1e-12);
  EXPECT_NEAR(out.pose.position.z, p_w.z(), 1e-12);
  const Eigen::Quaterniond q_wb(out.pose.orientation.w, out.pose.orientation.x,
                                out.pose.orientation.y, out.pose.orientation.z);
  EXPECT_LT((q_wb.toRotationMatrix() - (transform.R * q_cb.toRotationMatrix())).cwiseAbs().maxCoeff(),
            1e-12);

  // 协方差：Σ_W = J Σ_C Jᵀ。
  Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
  J.block<3, 3>(0, 0) = transform.R;
  J.block<3, 3>(3, 3) = transform.R;
  const Eigen::Matrix<double, 6, 6> expected = J * cov_c * J.transpose();
  const Eigen::Matrix<double, 6, 6> actual = fgb::covarianceFromMsg(out.covariance.elems);
  EXPECT_LT((actual - expected).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_TRUE(isSymmetric(actual, 1e-18));
  EXPECT_TRUE(isPositiveSemiDefinite(actual, 1e-15));
  // 只做对称化，不缩放：迹保持不变。
  EXPECT_NEAR(actual.trace(), cov_c.trace(), 1e-15);

  // 输入未被修改。
  const Eigen::Matrix<double, 6, 6> in_after = fgb::covarianceFromMsg(in.covariance.elems);
  EXPECT_DOUBLE_EQ((in_after - cov_c).cwiseAbs().maxCoeff(), 0.0);
}

/// 有限差分：把固定轴扰动输入经真实变换后，用蒙特卡洛统计验证 J Σ Jᵀ 与
/// 姿态误差映射 δθ_W ≈ R_a δθ。
TEST(PoseCovarianceTransform, FiniteDifferenceAttitudeErrorMapping)
{
  const Eigen::Vector3d gravity_camera =
      pitchRotation(kPitchDeg).transpose() * Eigen::Vector3d(0.0, 0.0, -kGravity);
  fgb::AlignmentTransform transform;
  ASSERT_TRUE(fgb::computeAlignmentTransform(gravity_camera, transform));

  // 使用较小的姿态方差，保证一阶线性化成立（二阶项可忽略）。
  Eigen::Matrix<double, 6, 6> cov_c = Eigen::Matrix<double, 6, 6>::Zero();
  cov_c.diagonal() << 4.0e-4, 9.0e-4, 1.6e-3, 4.0e-8, 9.0e-8, 1.6e-7;
  cov_c(0, 1) = cov_c(1, 0) = 1.0e-4;
  cov_c(0, 3) = cov_c(3, 0) = 2.0e-6;
  cov_c(2, 5) = cov_c(5, 2) = 3.0e-6;
  cov_c(3, 4) = cov_c(4, 3) = 4.0e-9;

  const Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(cov_c);
  ASSERT_EQ(llt.info(), Eigen::Success);
  const Eigen::Matrix<double, 6, 6> sqrt_cov = llt.matrixL();

  geometry_msgs::PoseWithCovariance nominal_in;
  nominal_in.pose.position.x = 0.4;
  nominal_in.pose.position.y = -0.3;
  nominal_in.pose.position.z = 1.2;
  const Eigen::Quaterniond q_cb(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()));
  nominal_in.pose.orientation.w = q_cb.w();
  nominal_in.pose.orientation.x = q_cb.x();
  nominal_in.pose.orientation.y = q_cb.y();
  nominal_in.pose.orientation.z = q_cb.z();
  fgb::covarianceToMsg(cov_c, nominal_in.covariance.elems);

  geometry_msgs::PoseWithCovariance nominal_out;
  ASSERT_TRUE(fgb::transformPoseWithCovariance(transform, nominal_in, nominal_out));

  Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
  J.block<3, 3>(0, 0) = transform.R;
  J.block<3, 3>(3, 3) = transform.R;
  const Eigen::Matrix<double, 6, 6> predicted = fgb::symmetrizeCovariance(J * cov_c * J.transpose());

  const int kSamples = 20000;
  std::mt19937 generator = makeGenerator();
  Eigen::Matrix<double, 6, 6> empirical = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> mean = Eigen::Matrix<double, 6, 1>::Zero();
  double max_rot_mapping_error = 0.0;
  double max_pos_mapping_error = 0.0;

  for (int k = 0; k < kSamples; ++k)
  {
    const Eigen::Matrix<double, 6, 1> delta = samplePerturbation(sqrt_cov, generator);

    // 输入侧扰动按 ROS 约定：位置直接相加，姿态是 camera_init 固定轴下的左乘扰动。
    geometry_msgs::PoseWithCovariance perturbed_in = nominal_in;
    perturbed_in.pose.position.x += delta(0);
    perturbed_in.pose.position.y += delta(1);
    perturbed_in.pose.position.z += delta(2);
    const Eigen::Vector3d rot_vec = delta.segment<3>(3);
    const double rot_angle = rot_vec.norm();
    const Eigen::Vector3d rot_axis =
        rot_angle > 0.0 ? Eigen::Vector3d(rot_vec / rot_angle) : Eigen::Vector3d::UnitX();
    const Eigen::Quaterniond rotation_error(Eigen::AngleAxisd(rot_angle, rot_axis));
    const Eigen::Quaterniond q_perturbed = (rotation_error * q_cb).normalized();
    perturbed_in.pose.orientation.w = q_perturbed.w();
    perturbed_in.pose.orientation.x = q_perturbed.x();
    perturbed_in.pose.orientation.y = q_perturbed.y();
    perturbed_in.pose.orientation.z = q_perturbed.z();

    geometry_msgs::PoseWithCovariance perturbed_out;
    ASSERT_TRUE(fgb::transformPoseWithCovariance(transform, perturbed_in, perturbed_out));

    // 位置映射：δp_W = R_a δp（精确成立）。
    const Eigen::Vector3d dp_world(perturbed_out.pose.position.x - nominal_out.pose.position.x,
                                   perturbed_out.pose.position.y - nominal_out.pose.position.y,
                                   perturbed_out.pose.position.z - nominal_out.pose.position.z);
    const Eigen::Vector3d dp_expected = transform.R * delta.segment<3>(0);
    max_pos_mapping_error = std::max(max_pos_mapping_error, (dp_world - dp_expected).norm());

    // 姿态映射：δθ_W ≈ R_a δθ（一阶）。
    const Eigen::Vector3d dtheta_world =
        fixedAxisRotationError(perturbed_out.pose.orientation, nominal_out.pose.orientation);
    const Eigen::Vector3d dtheta_expected = transform.R * delta.segment<3>(3);
    max_rot_mapping_error =
        std::max(max_rot_mapping_error, (dtheta_world - dtheta_expected).norm());

    Eigen::Matrix<double, 6, 1> sample;
    sample << dp_world, dtheta_world;
    mean += sample;
    empirical += sample * sample.transpose();
  }
  mean /= static_cast<double>(kSamples);
  empirical = empirical / static_cast<double>(kSamples) - mean * mean.transpose();

  EXPECT_LT(max_pos_mapping_error, 1e-12);
  EXPECT_LT(max_rot_mapping_error, 1e-5);
  EXPECT_LT(mean.norm(), 1e-3);

  // 经验协方差与 J Σ Jᵀ 逐元素比较。样本协方差元素的渐近标准差为
  // SE_ij = sqrt((Σ_ii Σ_jj + Σ_ij²) / N)，取 5 倍标准差作为判据
  // （36 个元素的最大偏差通常在 3 倍标准差量级）。
  double max_deviation_in_sigma = 0.0;
  int worst_i = -1;
  int worst_j = -1;
  for (int i = 0; i < 6; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      const double se = std::sqrt(
          (predicted(i, i) * predicted(j, j) + predicted(i, j) * predicted(i, j)) / kSamples);
      if (se <= 0.0) continue;
      const double deviation = std::abs(empirical(i, j) - predicted(i, j)) / se;
      if (deviation > max_deviation_in_sigma)
      {
        max_deviation_in_sigma = deviation;
        worst_i = i;
        worst_j = j;
      }
    }
  }
  EXPECT_LT(max_deviation_in_sigma, 5.0)
      << "最大偏差出现在 (" << worst_i << ", " << worst_j << ")";
}

/// 非半正定（不确定）协方差必须被拒绝：对角线非负不足以保证合法。
/// 复现用例：对角线全为 1、Σxz = Σzx = 2，最小特征值为 -1。
TEST(PoseCovarianceTransform, RejectsIndefiniteCovariance)
{
  const Eigen::Vector3d gravity_camera =
      pitchRotation(kPitchDeg).transpose() * Eigen::Vector3d(0.0, 0.0, -kGravity);
  fgb::AlignmentTransform transform;
  ASSERT_TRUE(fgb::computeAlignmentTransform(gravity_camera, transform));

  Eigen::Matrix<double, 6, 6> indefinite = Eigen::Matrix<double, 6, 6>::Identity();
  indefinite(0, 2) = indefinite(2, 0) = 2.0;  // 仍然对称，但不定

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(indefinite);
  EXPECT_NEAR(solver.eigenvalues().minCoeff(), -1.0, 1e-12);
  EXPECT_FALSE(fgb::covarianceIsValid(indefinite));

  geometry_msgs::PoseWithCovariance msg;
  msg.pose.orientation.w = 1.0;
  fgb::covarianceToMsg(indefinite, msg.covariance.elems);
  EXPECT_FALSE(fgb::poseCovarianceIsValid(msg.covariance.elems));

  geometry_msgs::PoseWithCovariance out;
  EXPECT_FALSE(fgb::transformPoseWithCovariance(transform, msg, out));

  // 该矩阵绕 Y 转 45 度后 Z 方差为负；确认输入检查在变换前就拦住了它。
  Eigen::Matrix<double, 6, 6> rotated = Eigen::Matrix<double, 6, 6>::Zero();
  const Eigen::Matrix3d r45 =
      Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
  rotated.block<3, 3>(0, 0) = r45 * indefinite.block<3, 3>(0, 0) * r45.transpose();
  rotated.block<3, 3>(3, 3) = indefinite.block<3, 3>(3, 3);
  EXPECT_LT(rotated(2, 2), 0.0);  // 变换后会出现的负方差
  EXPECT_FALSE(fgb::covarianceIsValid(rotated));
}

/// 结构性不对称的协方差必须被拒绝。
TEST(PoseCovarianceTransform, RejectsAsymmetricCovariance)
{
  Eigen::Matrix<double, 6, 6> asymmetric = makePositiveDefinite();
  asymmetric(0, 1) = 1.0e-3;  // 与 (1,0) 的 3.0e-3 不一致
  EXPECT_FALSE(fgb::covarianceIsValid(asymmetric));

  // 舍入量级的不对称（1e-12 相对）仍应接受：合法协方差不应被误杀。
  Eigen::Matrix<double, 6, 6> rounding = makePositiveDefinite();
  rounding(0, 1) += 1.0e-12 * rounding.cwiseAbs().maxCoeff();
  EXPECT_TRUE(fgb::covarianceIsValid(rounding));

  // 舍入量级的负特征值也应接受。
  Eigen::Matrix<double, 6, 6> nearly_psd = makePositiveDefinite();
  nearly_psd(3, 3) -= 1.0e-13 * nearly_psd.cwiseAbs().maxCoeff();
  EXPECT_TRUE(fgb::covarianceIsValid(nearly_psd));
}

/// 合法输入经变换后必须仍是合法协方差（对称、半正定）。
TEST(PoseCovarianceTransform, TransformedCovarianceStaysValid)
{
  const Eigen::Vector3d gravity_camera =
      pitchRotation(kPitchDeg).transpose() * Eigen::Vector3d(0.0, 0.0, -kGravity);
  fgb::AlignmentTransform transform;
  ASSERT_TRUE(fgb::computeAlignmentTransform(gravity_camera, transform));

  geometry_msgs::PoseWithCovariance in;
  in.pose.orientation.w = 1.0;
  // 量级差别很大的对角协方差 + 交叉项：变换后仍应合法。
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
  cov.diagonal() << 1.0e-6, 1.0e-3, 5.0e-1, 1.0e-9, 1.0e-6, 1.0e-4;
  cov(0, 1) = cov(1, 0) = 1.0e-5;
  cov(2, 5) = cov(5, 2) = 1.0e-3;
  fgb::covarianceToMsg(cov, in.covariance.elems);

  geometry_msgs::PoseWithCovariance out;
  ASSERT_TRUE(fgb::transformPoseWithCovariance(transform, in, out));
  const Eigen::Matrix<double, 6, 6> out_cov = fgb::covarianceFromMsg(out.covariance.elems);
  EXPECT_TRUE(fgb::covarianceIsValid(out_cov));
  EXPECT_LT((out_cov - out_cov.transpose()).cwiseAbs().maxCoeff(), 1e-18);
}

/// 非法输入必须被拒绝：非有限位置、无效四元数、非法协方差。
TEST(PoseCovarianceTransform, InvalidInputsAreRejected)
{
  const Eigen::Vector3d gravity_camera =
      pitchRotation(kPitchDeg).transpose() * Eigen::Vector3d(0.0, 0.0, -kGravity);
  fgb::AlignmentTransform transform;
  ASSERT_TRUE(fgb::computeAlignmentTransform(gravity_camera, transform));

  geometry_msgs::PoseWithCovariance valid;
  valid.pose.orientation.w = 1.0;
  geometry_msgs::PoseWithCovariance out;

  geometry_msgs::PoseWithCovariance bad_position = valid;
  bad_position.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(fgb::transformPoseWithCovariance(transform, bad_position, out));

  geometry_msgs::PoseWithCovariance zero_quaternion = valid;
  zero_quaternion.pose.orientation.w = 0.0;
  EXPECT_FALSE(fgb::transformPoseWithCovariance(transform, zero_quaternion, out));

  geometry_msgs::PoseWithCovariance tiny_quaternion = valid;
  tiny_quaternion.pose.orientation.w = 1e-9;
  EXPECT_FALSE(fgb::transformPoseWithCovariance(transform, tiny_quaternion, out));

  geometry_msgs::PoseWithCovariance nan_covariance = valid;
  nan_covariance.covariance[7] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(fgb::transformPoseWithCovariance(transform, nan_covariance, out));

  geometry_msgs::PoseWithCovariance negative_variance = valid;
  negative_variance.covariance[14] = -1.0;
  EXPECT_FALSE(fgb::transformPoseWithCovariance(transform, negative_variance, out));

  // 全零协方差（FAST-LIO2 未提供速度时的合法输入）应通过。
  geometry_msgs::PoseWithCovariance zero_covariance = valid;
  EXPECT_TRUE(fgb::transformPoseWithCovariance(transform, zero_covariance, out));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
