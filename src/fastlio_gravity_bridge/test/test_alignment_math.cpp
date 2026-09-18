// 数学测试：重力对齐旋转的构造、左乘方向，以及"重力最终沿 -Z"。
#include <cmath>

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fastlio_gravity_bridge/gravity_alignment.h"

namespace fgb = fastlio_gravity_bridge;

namespace
{

constexpr double kGravity = 9.81;
constexpr double kTol = 1e-12;

/// 20.12 度前倾：机体系到世界系绕 +Y 转 +20.12 度（前倾 = 机头向下）。
Eigen::Matrix3d pitchBodyToWorld(double pitch_deg)
{
  return Eigen::AngleAxisd(pitch_deg * M_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

void expectQuaternionNear(const Eigen::Quaterniond &actual, const Eigen::Quaterniond &expected,
                          double tol)
{
  // q 与 -q 表示同一旋转，比较时取符号一致的方向。
  const Eigen::Quaterniond a = actual.dot(expected) < 0.0
                                   ? Eigen::Quaterniond(-actual.w(), -actual.x(), -actual.y(),
                                                        -actual.z())
                                   : actual;
  EXPECT_NEAR(a.w(), expected.w(), tol);
  EXPECT_NEAR(a.x(), expected.x(), tol);
  EXPECT_NEAR(a.y(), expected.y(), tol);
  EXPECT_NEAR(a.z(), expected.z(), tol);
}

void expectRotationNear(const Eigen::Matrix3d &actual, const Eigen::Matrix3d &expected, double tol)
{
  EXPECT_LT((actual - expected).cwiseAbs().maxCoeff(), tol);
}

}  // namespace

/// 单位旋转：重力沿相机系 -Z 时，对齐旋转应为单位四元数。
TEST(GravityAlignmentMath, IdentityGravityGivesIdentityRotation)
{
  fgb::AlignmentTransform transform;
  ASSERT_TRUE(fgb::computeAlignmentTransform(Eigen::Vector3d(0.0, 0.0, -kGravity), transform));
  ASSERT_TRUE(transform.valid);

  expectQuaternionNear(transform.q, Eigen::Quaterniond::Identity(), 1e-15);
  expectRotationNear(transform.R, Eigen::Matrix3d::Identity(), 1e-15);

  const Eigen::Vector3d g_aligned = transform.R * Eigen::Vector3d(0.0, 0.0, -kGravity);
  EXPECT_LT((g_aligned - Eigen::Vector3d(0.0, 0.0, -kGravity)).norm(), kTol);
}

/// 约 20.12 度前倾：对齐后重力沿 world 的 -Z，且旋转等于绕 +Y 转 20.12 度（最小旋转、不动 yaw）。
TEST(GravityAlignmentMath, PitchTiltIsAlignedToNegativeZ)
{
  const double pitch_deg = 20.12;
  const Eigen::Matrix3d R_world_to_body = pitchBodyToWorld(pitch_deg).transpose();
  const Eigen::Vector3d gravity_camera = R_world_to_body * Eigen::Vector3d(0.0, 0.0, -kGravity);

  // 静态时机体系测得的比力为 -g，FAST-LIO2 发布的 grav 即上式结果。
  EXPECT_NEAR(gravity_camera.x(), kGravity * std::sin(pitch_deg * M_PI / 180.0), 1e-12);
  EXPECT_NEAR(gravity_camera.y(), 0.0, 1e-12);
  EXPECT_NEAR(gravity_camera.z(), -kGravity * std::cos(pitch_deg * M_PI / 180.0), 1e-12);

  fgb::AlignmentTransform transform;
  ASSERT_TRUE(fgb::computeAlignmentTransform(gravity_camera, transform));

  const Eigen::Vector3d g_dir = gravity_camera.normalized();
  const Eigen::Vector3d g_in_world = transform.R * g_dir;
  EXPECT_LT(std::abs(g_in_world.x()), kTol);
  EXPECT_LT(std::abs(g_in_world.y()), kTol);
  EXPECT_NEAR(g_in_world.z(), -1.0, kTol);

  // 最小旋转：绕 +Y 转 +20.12 度，与给定倾角一致（含 R_y(20.12) 的三维数值）。
  expectRotationNear(transform.R, pitchBodyToWorld(pitch_deg), 1e-12);
  const Eigen::Quaterniond expected(Eigen::AngleAxisd(pitch_deg * M_PI / 180.0,
                                                      Eigen::Vector3d::UnitY()));
  expectQuaternionNear(transform.q, expected, 1e-12);

  // 相机系的"上"方向（-重力方向）在 world 下应为 +Z。
  const Eigen::Vector3d up_world = transform.R * (-g_dir);
  EXPECT_NEAR(up_world.z(), 1.0, kTol);
  EXPECT_LT(std::abs(up_world.x()), kTol);
  EXPECT_LT(std::abs(up_world.y()), kTol);
}

/// 位置必须左乘 R_a（p_W = R_a p_C），而不是右乘或转置。
TEST(GravityAlignmentMath, PositionUsesLeftMultiplication)
{
  const double pitch_deg = 20.12;
  const Eigen::Vector3d gravity_camera =
      pitchBodyToWorld(pitch_deg).transpose() * Eigen::Vector3d(0.0, 0.0, -kGravity);

  fgb::AlignmentTransform transform;
  ASSERT_TRUE(fgb::computeAlignmentTransform(gravity_camera, transform));

  // 前倾 20 度时机体系看到的"水平前推 d"在 world 下应是沿 +X 的纯水平位移。
  const double d = 3.0;
  const Eigen::Vector3d p_camera(d * std::cos(pitch_deg * M_PI / 180.0), 0.0,
                                 d * std::sin(pitch_deg * M_PI / 180.0));
  const Eigen::Vector3d p_world = transform.R * p_camera;
  EXPECT_NEAR(p_world.x(), d, 1e-12);
  EXPECT_NEAR(p_world.y(), 0.0, 1e-12);
  EXPECT_NEAR(p_world.z(), 0.0, 1e-12);

  // 转置方向要明显不同，避免把右乘/转置当成正确实现。
  const Eigen::Vector3d p_wrong = transform.R.transpose() * p_camera;
  EXPECT_GT((p_world - p_wrong).norm(), 1.0);
}

/// 姿态必须左乘 q_a（q_WB = q_a ⊗ q_CB），而不是右乘。
TEST(GravityAlignmentMath, OrientationUsesLeftMultiplication)
{
  const double pitch_deg = 20.12;
  const Eigen::Vector3d gravity_camera =
      pitchBodyToWorld(pitch_deg).transpose() * Eigen::Vector3d(0.0, 0.0, -kGravity);

  fgb::AlignmentTransform transform;
  ASSERT_TRUE(fgb::computeAlignmentTransform(gravity_camera, transform));

  const Eigen::Quaterniond q_cb(Eigen::AngleAxisd(30.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()));
  const Eigen::Quaterniond q_wb = transform.q * q_cb;

  // 与显式矩阵乘积对照（左乘 = 世界侧复合）。
  expectRotationNear(q_wb.toRotationMatrix(),
                     transform.R * q_cb.toRotationMatrix(), 1e-12);

  // 右乘结果不同，确保实现方向正确。
  const Eigen::Quaterniond q_right = q_cb * transform.q;
  EXPECT_GT((q_wb.toRotationMatrix() - q_right.toRotationMatrix()).cwiseAbs().maxCoeff(), 1e-3);
}

/// 反向重力：连续两个 180 度旋转回到自身，且重力仍沿 -Z。
TEST(GravityAlignmentMath, ReversedGravityIsAlignedAndIsProperRotation)
{
  fgb::AlignmentTransform transform;
  ASSERT_TRUE(fgb::computeAlignmentTransform(Eigen::Vector3d(0.0, 0.0, kGravity), transform));

  const Eigen::Vector3d g_aligned = transform.R * Eigen::Vector3d(0.0, 0.0, 1.0);
  EXPECT_LT(std::abs(g_aligned.x()), kTol);
  EXPECT_LT(std::abs(g_aligned.y()), kTol);
  EXPECT_NEAR(g_aligned.z(), -1.0, kTol);

  EXPECT_NEAR(std::abs(transform.q.w()), 0.0, 1e-12);
  EXPECT_NEAR(transform.q.norm(), 1.0, 1e-15);
  EXPECT_NEAR(transform.R.determinant(), 1.0, 1e-12);
  expectRotationNear(transform.R * transform.R, Eigen::Matrix3d::Identity(), 1e-12);
}

/// 一般倾斜（含 yaw）：只做最小旋转，旋转角等于重力方向与 -Z 的夹角。
TEST(GravityAlignmentMath, ArbitraryTiltUsesMinimalRotation)
{
  const Eigen::Matrix3d R_world_to_body =
      (Eigen::AngleAxisd(12.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-8.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  const Eigen::Vector3d gravity_camera = R_world_to_body * Eigen::Vector3d(0.0, 0.0, -kGravity);

  fgb::AlignmentTransform transform;
  ASSERT_TRUE(fgb::computeAlignmentTransform(gravity_camera, transform));

  const Eigen::Vector3d g_dir = gravity_camera.normalized();
  const Eigen::Vector3d g_in_world = transform.R * g_dir;
  EXPECT_LT((g_in_world - Eigen::Vector3d(0.0, 0.0, -1.0)).norm(), kTol);

  const double expected_angle = std::acos(
      std::max(-1.0, std::min(1.0, g_dir.dot(-Eigen::Vector3d::UnitZ()))));
  const double actual_angle = Eigen::AngleAxisd(transform.R).angle();
  EXPECT_NEAR(actual_angle, expected_angle, 1e-12);
}

/// 非法输入：零向量、非有限值都必须被拒绝，且不修改输出。
TEST(GravityAlignmentMath, InvalidGravityIsRejected)
{
  fgb::AlignmentTransform transform;
  EXPECT_FALSE(fgb::computeAlignmentTransform(Eigen::Vector3d::Zero(), transform));
  EXPECT_FALSE(transform.valid);
  EXPECT_FALSE(fgb::computeAlignmentTransform(Eigen::Vector3d(1e-9, 0.0, 0.0), transform));
  EXPECT_FALSE(fgb::computeAlignmentTransform(
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, -kGravity), transform));
  EXPECT_FALSE(fgb::computeAlignmentTransform(
      Eigen::Vector3d(0.0, std::numeric_limits<double>::infinity(), -kGravity), transform));
  EXPECT_FALSE(transform.valid);
}

/// 协方差变换在未锁定对齐旋转时必须失败。
TEST(GravityAlignmentMath, TransformRequiresLockedAlignment)
{
  fgb::AlignmentTransform transform;
  geometry_msgs::PoseWithCovariance in;
  in.pose.orientation.w = 1.0;
  geometry_msgs::PoseWithCovariance out;
  EXPECT_FALSE(fgb::transformPoseWithCovariance(transform, in, out));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
