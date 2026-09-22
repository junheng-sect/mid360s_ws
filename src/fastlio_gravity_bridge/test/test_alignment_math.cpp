// 数学测试：重力对齐旋转的构造、左乘方向，以及"重力最终沿 -Z"。
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fastlio_gravity_bridge/gravity_alignment.h"

namespace fgb = fastlio_gravity_bridge;

namespace
{

constexpr double kGravity = 9.81;
constexpr double kTol = 1e-12;
/// 节点默认的 mounting_pitch_deg（雷达相对机体前倾角），fixed 模式用它做安装补偿。
constexpr double kMountPitchDeg = 20.0;
constexpr double kDeg = M_PI / 180.0;

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

// --------------------------------------------------------------------------
// 固定安装补偿（mounting_source=fixed）用的"真实姿态"生成器。
// 它们只按物理关系独立推导，不调用实现里的安装/对齐公式，用来避免"复写实现"。
// --------------------------------------------------------------------------

/// 真实安装：雷达相对机体前倾 mount_deg（机头向下），R_base<-lidar = R_y(+mount_deg)。
Eigen::Matrix3d baseFromLidar(double mount_deg)
{
  return Eigen::AngleAxisd(mount_deg * kDeg, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

/// 独立生成相机系（camera_init）下的雷达姿态：
/// camera_init 与初始化时刻的雷达系重合，且 R_W<-lidar = R_wb · R_base<-lidar，于是
///   R_C<-lidar(t) = (R_W<-lidar(t0))ᵀ · R_W<-lidar(t)
Eigen::Quaterniond cameraInitFromLidar(const Eigen::Matrix3d &R_wb_init,
                                       const Eigen::Matrix3d &R_wb_now, double mount_deg)
{
  const Eigen::Matrix3d R_w_lidar_init = R_wb_init * baseFromLidar(mount_deg);
  const Eigen::Matrix3d R_w_lidar_now = R_wb_now * baseFromLidar(mount_deg);
  return Eigen::Quaterniond(R_w_lidar_init.transpose() * R_w_lidar_now).normalized();
}

/// 同一物理模型下的初始化重力（camera_init 下）：静止时机体只受重力，FAST-LIO 发布的
/// grav 就是世界重力 (0,0,-g) 在初始化时刻雷达系里的表达。
Eigen::Vector3d gravityFromBodyAttitude(const Eigen::Matrix3d &R_wb_init, double mount_deg)
{
  return (R_wb_init * baseFromLidar(mount_deg)).transpose() *
         Eigen::Vector3d(0.0, 0.0, -kGravity);
}

/// 姿态相对真实世界竖直方向（+Z）的倾角（度）：绕竖直轴的偏航不影响它。
double tiltFromVerticalDeg(const Eigen::Vector3d &up)
{
  const Eigen::Vector3d n = up.normalized();
  return std::acos(std::max(-1.0, std::min(1.0, n.z()))) * 180.0 / M_PI;
}

/// 走一遍实现的位姿变换，只取输出姿态。
Eigen::Quaterniond transformOrientation(const fgb::AlignmentTransform &transform,
                                        const Eigen::Quaterniond &q_ib,
                                        const Eigen::Quaterniond &q_cb)
{
  geometry_msgs::PoseWithCovariance in;
  in.pose.orientation.w = q_cb.w();
  in.pose.orientation.x = q_cb.x();
  in.pose.orientation.y = q_cb.y();
  in.pose.orientation.z = q_cb.z();
  geometry_msgs::PoseWithCovariance out;
  EXPECT_TRUE(fgb::transformPoseWithCovariance(transform, q_ib, in, out));
  return Eigen::Quaterniond(out.pose.orientation.w, out.pose.orientation.x,
                            out.pose.orientation.y, out.pose.orientation.z);
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
  EXPECT_FALSE(fgb::transformPoseWithCovariance(transform, Eigen::Quaterniond::Identity(), in,
                                                out));
}

/// 【旧 gravity 模式】安装旋转：初始化时机体水平时，q_IB = q_a⁻¹ 应把机体姿态还原成水平。
/// 该模式把初始化时测到的安装倾角（这里取 20.12 度）整体抵消，前提是初始化时机体水平；
/// 默认已改为固定安装补偿（q_IB = Ry(-mounting_pitch_deg)），见下面的 FixedMounting* 用例。
TEST(GravityAlignmentMath, MountingCancelsInitialTilt)
{
  const double pitch_deg = 20.12;
  // 机体水平、雷达前倾 20.12 度时，camera_init 下测到的重力
  const Eigen::Vector3d gravity_camera =
      pitchBodyToWorld(pitch_deg).transpose() * Eigen::Vector3d(0.0, 0.0, -kGravity);

  fgb::AlignmentTransform transform;
  ASSERT_TRUE(fgb::computeAlignmentTransform(gravity_camera, transform));

  const Eigen::Quaterniond q_ib = fgb::mountingFromAlignment(transform);
  EXPECT_NEAR(q_ib.norm(), 1.0, 1e-15);
  // 安装旋转恰好抵消安装倾角：绕 +Y 转 -20.12 度
  expectRotationNear(q_ib.toRotationMatrix(), pitchBodyToWorld(-pitch_deg), 1e-12);

  // 初始化时刻 q_CI = I -> 输出姿态应为单位阵（机体水平），而不是前倾 20 度
  geometry_msgs::PoseWithCovariance in;
  in.pose.position.z = 1.0;
  in.pose.orientation.w = 1.0;  // q_CI = I
  geometry_msgs::PoseWithCovariance out;
  ASSERT_TRUE(fgb::transformPoseWithCovariance(transform, q_ib, in, out));

  const Eigen::Quaterniond q_out(out.pose.orientation.w, out.pose.orientation.x,
                                 out.pose.orientation.y, out.pose.orientation.z);
  expectQuaternionNear(q_out, Eigen::Quaterniond::Identity(), 1e-12);

  // 不做安装补偿（单位四元数）时，输出仍是前倾 20.12 度的 IMU 姿态
  geometry_msgs::PoseWithCovariance out_raw;
  ASSERT_TRUE(fgb::transformPoseWithCovariance(transform, Eigen::Quaterniond::Identity(),
                                               in, out_raw));
  const Eigen::Quaterniond q_raw(out_raw.pose.orientation.w, out_raw.pose.orientation.x,
                                 out_raw.pose.orientation.y, out_raw.pose.orientation.z);
  expectQuaternionNear(q_raw, Eigen::Quaterniond(Eigen::AngleAxisd(
                                  pitch_deg * M_PI / 180.0, Eigen::Vector3d::UnitY())),
                       1e-12);
  // 两者恰好相差一个安装旋转
  expectRotationNear((q_raw.conjugate() * q_out).toRotationMatrix(), q_ib.toRotationMatrix(),
                     1e-12);

  // 位置只受世界对齐旋转影响，与安装旋转无关：加不加补偿结果相同
  EXPECT_DOUBLE_EQ(out.pose.position.x, out_raw.pose.position.x);
  EXPECT_DOUBLE_EQ(out.pose.position.y, out_raw.pose.position.y);
  EXPECT_DOUBLE_EQ(out.pose.position.z, out_raw.pose.position.z);
  // 而对齐旋转确实会把位置转到 world（z 不再是 1）
  const Eigen::Vector3d p_expected = transform.R * Eigen::Vector3d(0.0, 0.0, 1.0);
  EXPECT_NEAR(out.pose.position.z, p_expected.z(), 1e-15);
  EXPECT_NEAR(out.pose.position.x, p_expected.x(), 1e-15);
  EXPECT_GT(std::abs(out.pose.position.x), 0.3);
}

/// 【旧 gravity 模式】安装补偿后，机体做纯俯仰 / 纯偏航时读数如实反映，不被安装角污染。
TEST(GravityAlignmentMath, MountingKeepsBodyAttitudeFaithful)
{
  const double mount_deg = 20.12;
  const Eigen::Vector3d gravity_camera =
      pitchBodyToWorld(mount_deg).transpose() * Eigen::Vector3d(0.0, 0.0, -kGravity);

  fgb::AlignmentTransform transform;
  ASSERT_TRUE(fgb::computeAlignmentTransform(gravity_camera, transform));
  const Eigen::Quaterniond q_ib = fgb::mountingFromAlignment(transform);

  // 机体在水平基础上俯仰 φ：q_CI = q_a⁻¹ ⊗ q_WB ⊗ q_BI，这里直接构造 q_CI。
  // q_WI = q_WB ⊗ q_BI，而 q_BI 的矩阵即是 q_a（机体水平时），故 q_CI = q_a⁻¹ ⊗ q_WI。
  const Eigen::Quaterniond q_a = transform.q;
  for (const auto &body_rotation :
       {Eigen::AngleAxisd(10.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()),   // 俯仰
        Eigen::AngleAxisd(-7.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()),   // 横滚
        Eigen::AngleAxisd(25.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ())})  // 偏航
  {
    const Eigen::Quaterniond q_wb(body_rotation);
    const Eigen::Quaterniond q_wi = q_wb * q_a;
    const Eigen::Quaterniond q_ci = q_a.conjugate() * q_wi;

    geometry_msgs::PoseWithCovariance in;
    in.pose.orientation.w = q_ci.w();
    in.pose.orientation.x = q_ci.x();
    in.pose.orientation.y = q_ci.y();
    in.pose.orientation.z = q_ci.z();
    geometry_msgs::PoseWithCovariance out;
    ASSERT_TRUE(fgb::transformPoseWithCovariance(transform, q_ib, in, out));

    const Eigen::Quaterniond q_out(out.pose.orientation.w, out.pose.orientation.x,
                                   out.pose.orientation.y, out.pose.orientation.z);
    expectRotationNear(q_out.toRotationMatrix(), q_wb.toRotationMatrix(), 1e-12);
  }
}

/// mountingFromAlignment 在未锁定时返回单位四元数。
TEST(GravityAlignmentMath, MountingIsIdentityWhenUnlocked)
{
  fgb::AlignmentTransform transform;  // valid = false
  expectQuaternionNear(fgb::mountingFromAlignment(transform), Eigen::Quaterniond::Identity(),
                       0.0);
}

// ---------------------------------------------------------------------------
// 以下为固定安装补偿（mounting_source=fixed，默认）的验收用例。
// ---------------------------------------------------------------------------

/// 固定安装旋转就是"雷达前倾角"的逆：q_IB = R_y(-mounting_pitch_deg)。
TEST(GravityAlignmentMath, FixedMountingInvertsLidarForwardTilt)
{
  Eigen::Quaterniond q_ib;
  ASSERT_TRUE(fgb::computeFixedMounting(kMountPitchDeg, q_ib));
  EXPECT_NEAR(q_ib.norm(), 1.0, 1e-15);
  // 雷达绕 +Y 前倾 20 度 -> 右乘补偿绕 +Y 转 -20 度
  expectRotationNear(q_ib.toRotationMatrix(), pitchBodyToWorld(-kMountPitchDeg), 1e-15);

  // 0 度等于不做补偿；后仰安装（负角度）取相反的逆。
  Eigen::Quaterniond zero;
  ASSERT_TRUE(fgb::computeFixedMounting(0.0, zero));
  expectQuaternionNear(zero, Eigen::Quaterniond::Identity(), 1e-15);
  Eigen::Quaterniond tilted_back;
  ASSERT_TRUE(fgb::computeFixedMounting(-8.0, tilted_back));
  expectRotationNear(tilted_back.toRotationMatrix(), pitchBodyToWorld(8.0), 1e-15);

  // 非有限角度必须被拒绝，且不修改输出。
  const Eigen::Quaterniond keep = zero;
  EXPECT_FALSE(fgb::computeFixedMounting(std::numeric_limits<double>::quiet_NaN(), zero));
  EXPECT_FALSE(fgb::computeFixedMounting(std::numeric_limits<double>::infinity(), zero));
  EXPECT_FALSE(fgb::computeFixedMounting(-std::numeric_limits<double>::infinity(), zero));
  expectQuaternionNear(zero, keep, 0.0);
}

/// 核心验收：初始化时机体不水平时，输出必须保留真实倾角，不再像旧 gravity 模式那样
/// 把初始倾角当作安装角一并抵消（统一归零）。
/// 真实姿态、初始化重力与 /Odometry 姿态都由上面的物理模型独立生成。
TEST(GravityAlignmentMath, FixedMountingKeepsActualTiltAtTiltedStart)
{
  struct StartCase
  {
    std::string name;
    Eigen::Matrix3d R_wb;    ///< 初始化时机体在真实世界系下的姿态
    bool gauge_is_identity;  ///< 该初始姿态下 world 的偏航基准恰好等于真实世界
  };
  const std::vector<StartCase> cases = {
      {"机体水平", Eigen::Matrix3d::Identity(), true},
      {"前倾 10 度", pitchBodyToWorld(10.0), true},
      {"后仰 10 度", pitchBodyToWorld(-10.0), true},
      {"横滚 15 度", Eigen::AngleAxisd(15.0 * kDeg, Eigen::Vector3d::UnitX()).toRotationMatrix(),
       false},
  };

  Eigen::Quaterniond q_ib;
  ASSERT_TRUE(fgb::computeFixedMounting(kMountPitchDeg, q_ib));

  for (const StartCase &c : cases)
  {
    fgb::AlignmentTransform transform;
    ASSERT_TRUE(fgb::computeAlignmentTransform(
        gravityFromBodyAttitude(c.R_wb, kMountPitchDeg), transform));

    // 初始化时刻 q_CI = I（camera_init 就是初始化时刻的雷达系）。
    const Eigen::Matrix3d R_out =
        transformOrientation(transform, q_ib, Eigen::Quaterniond::Identity()).toRotationMatrix();

    // (1) 保留真实倾角：输出相对世界竖直的倾角 == 机体真实倾角（旧模式这里恒为 0）。
    EXPECT_NEAR(tiltFromVerticalDeg(R_out * Eigen::Vector3d::UnitZ()),
                tiltFromVerticalDeg(c.R_wb * Eigen::Vector3d::UnitZ()), 1e-12)
        << c.name;
    if (!c.gauge_is_identity)
    {
      EXPECT_GT(tiltFromVerticalDeg(R_out * Eigen::Vector3d::UnitZ()), 1.0) << c.name;
    }

    // (2) 重力对齐定不了航向，唯一的不确定量是绕竖直轴的固定偏航：R_out = R_z(psi)·R_wb。
    const Eigen::Matrix3d gauge = R_out * c.R_wb.transpose();
    EXPECT_LT((gauge * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitZ()).norm(), 1e-12)
        << c.name;
    EXPECT_LT(std::abs(gauge(2, 0)) + std::abs(gauge(2, 1)) + std::abs(gauge(0, 2)) +
                  std::abs(gauge(1, 2)),
              1e-12)
        << c.name;
    if (c.gauge_is_identity) expectRotationNear(gauge, Eigen::Matrix3d::Identity(), 1e-12);
  }
}

/// 纯俯仰启动（world 偏航基准与真实世界一致）时，输出完全跟随真实机体姿态：
/// 之后恢复水平、纯偏航、纯俯仰、纯横滚、组合旋转都要逐一复现。
TEST(GravityAlignmentMath, FixedMountingTracksTrueAttitudeAfterPitchStart)
{
  const Eigen::Matrix3d R_wb_init = pitchBodyToWorld(10.0);
  fgb::AlignmentTransform transform;
  ASSERT_TRUE(
      fgb::computeAlignmentTransform(gravityFromBodyAttitude(R_wb_init, kMountPitchDeg), transform));
  Eigen::Quaterniond q_ib;
  ASSERT_TRUE(fgb::computeFixedMounting(kMountPitchDeg, q_ib));

  const std::vector<std::pair<std::string, Eigen::Matrix3d>> later = {
      {"倾斜启动后恢复水平", Eigen::Matrix3d::Identity()},
      {"纯偏航 25 度", Eigen::AngleAxisd(25.0 * kDeg, Eigen::Vector3d::UnitZ()).toRotationMatrix()},
      {"纯俯仰 -10 度", pitchBodyToWorld(-10.0)},
      {"纯横滚 12 度", Eigen::AngleAxisd(12.0 * kDeg, Eigen::Vector3d::UnitX()).toRotationMatrix()},
      {"偏航-俯仰-横滚组合",
       (Eigen::AngleAxisd(-40.0 * kDeg, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(8.0 * kDeg, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(-6.0 * kDeg, Eigen::Vector3d::UnitX()))
           .toRotationMatrix()},
  };

  for (const auto &c : later)
  {
    const Eigen::Quaterniond q_out = transformOrientation(
        transform, q_ib, cameraInitFromLidar(R_wb_init, c.second, kMountPitchDeg));
    expectRotationNear(q_out.toRotationMatrix(), c.second, 1e-12);
  }
}

/// 横滚倾斜启动时 world 的偏航基准与真实世界有一个固定偏差，但：
///  - 输出的真实倾角仍然保留；
///  - 任意两次输出之间的相对旋转（含偏航）与真实机体的相对旋转完全一致；
///  - 该偏差在整个过程中保持不变（不会漂移）。
TEST(GravityAlignmentMath, FixedMountingReproducesRelativeRotationFromRollStart)
{
  const Eigen::Matrix3d R_wb_init =
      Eigen::AngleAxisd(15.0 * kDeg, Eigen::Vector3d::UnitX()).toRotationMatrix();
  fgb::AlignmentTransform transform;
  ASSERT_TRUE(
      fgb::computeAlignmentTransform(gravityFromBodyAttitude(R_wb_init, kMountPitchDeg), transform));
  Eigen::Quaterniond q_ib;
  ASSERT_TRUE(fgb::computeFixedMounting(kMountPitchDeg, q_ib));

  const Eigen::Quaterniond q_out_init =
      transformOrientation(transform, q_ib, Eigen::Quaterniond::Identity());
  const Eigen::Matrix3d gauge = q_out_init.toRotationMatrix() * R_wb_init.transpose();
  // 初始偏航偏差非零（横滚启动时最小对齐旋转会带出航向偏差），只能是绕竖直轴的旋转。
  EXPECT_GT(std::abs(std::atan2(gauge(1, 0), gauge(0, 0))), 1e-3);

  const std::vector<std::pair<std::string, Eigen::Matrix3d>> later = {
      {"保持横滚", R_wb_init},
      {"横滚基础上偏航 30 度",
       Eigen::AngleAxisd(30.0 * kDeg, Eigen::Vector3d::UnitZ()) * R_wb_init},
      {"恢复水平", Eigen::Matrix3d::Identity()},
      {"组合旋转",
       (Eigen::AngleAxisd(20.0 * kDeg, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(-9.0 * kDeg, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(14.0 * kDeg, Eigen::Vector3d::UnitX()))
           .toRotationMatrix()},
  };

  for (const auto &c : later)
  {
    const Eigen::Quaterniond q_out = transformOrientation(
        transform, q_ib, cameraInitFromLidar(R_wb_init, c.second, kMountPitchDeg));
    const Eigen::Matrix3d R_out = q_out.toRotationMatrix();

    // 相对旋转忠实复现（偏航也在内）：世界侧相对量 = R_out(t0)ᵀ R_out(t)。
    expectRotationNear((q_out_init.conjugate() * q_out).toRotationMatrix(),
                       R_wb_init.transpose() * c.second, 1e-12);
    // 偏航基准固定不变。
    expectRotationNear(R_out * c.second.transpose(), gauge, 1e-12);
    // 倾角如实反映真实机体姿态。
    EXPECT_NEAR(tiltFromVerticalDeg(R_out * Eigen::Vector3d::UnitZ()),
                tiltFromVerticalDeg(c.second * Eigen::Vector3d::UnitZ()), 1e-12);
  }
}

/// 回归检查：同一份重力与位置/姿态/协方差输入下，只换安装补偿不改变输出位置与协方差；
/// 姿态只相差一个右乘的安装旋转；三种来源（fixed / gravity / none）都给出合法协方差。
TEST(GravityAlignmentMath, MountingChoiceDoesNotChangePositionOrCovariance)
{
  // 用 20.12 度的安装倾角生成初始化重力：与本机固定补偿的 20.0 度刻意不同，
  // 保证 fixed 与旧 gravity 模式给出的安装旋转确实不一样。
  const Eigen::Matrix3d R_wb_init = Eigen::Matrix3d::Identity();
  fgb::AlignmentTransform transform;
  ASSERT_TRUE(fgb::computeAlignmentTransform(gravityFromBodyAttitude(R_wb_init, 20.12), transform));

  Eigen::Quaterniond q_fixed;
  ASSERT_TRUE(fgb::computeFixedMounting(kMountPitchDeg, q_fixed));
  const Eigen::Quaterniond q_gravity = fgb::mountingFromAlignment(transform);
  const Eigen::Quaterniond q_none = Eigen::Quaterniond::Identity();
  EXPECT_GT((q_fixed.toRotationMatrix() - q_gravity.toRotationMatrix()).cwiseAbs().maxCoeff(),
            0.1 * kDeg);
  EXPECT_LT(
      Eigen::AngleAxisd(q_fixed.conjugate() * q_gravity).angle() * 180.0 / M_PI, 1.0);

  geometry_msgs::PoseWithCovariance in;
  in.pose.position.x = 1.5;
  in.pose.position.y = -0.25;
  in.pose.position.z = 2.0;
  const Eigen::Quaterniond q_cb(Eigen::AngleAxisd(0.35, Eigen::Vector3d(1.0, -2.0, 0.5).normalized()));
  in.pose.orientation.w = q_cb.w();
  in.pose.orientation.x = q_cb.x();
  in.pose.orientation.y = q_cb.y();
  in.pose.orientation.z = q_cb.z();
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
  cov.diagonal() << 1.0e-2, 4.0e-2, 9.0e-2, 1.0e-5, 4.0e-5, 9.0e-5;
  cov(0, 1) = cov(1, 0) = 3.0e-3;
  cov(2, 5) = cov(5, 2) = 1.0e-4;
  fgb::covarianceToMsg(cov, in.covariance.elems);

  geometry_msgs::PoseWithCovariance out_fixed;
  geometry_msgs::PoseWithCovariance out_gravity;
  geometry_msgs::PoseWithCovariance out_none;
  ASSERT_TRUE(fgb::transformPoseWithCovariance(transform, q_fixed, in, out_fixed));
  ASSERT_TRUE(fgb::transformPoseWithCovariance(transform, q_gravity, in, out_gravity));
  ASSERT_TRUE(fgb::transformPoseWithCovariance(transform, q_none, in, out_none));

  // 位置与协方差与安装补偿无关（逐位相同），协方差依旧合法。
  EXPECT_DOUBLE_EQ(out_fixed.pose.position.x, out_none.pose.position.x);
  EXPECT_DOUBLE_EQ(out_fixed.pose.position.y, out_none.pose.position.y);
  EXPECT_DOUBLE_EQ(out_fixed.pose.position.z, out_none.pose.position.z);
  EXPECT_DOUBLE_EQ(out_fixed.pose.position.x, out_gravity.pose.position.x);
  for (int i = 0; i < 36; ++i)
  {
    EXPECT_DOUBLE_EQ(out_fixed.covariance.elems[i], out_none.covariance.elems[i]) << "index " << i;
    EXPECT_DOUBLE_EQ(out_fixed.covariance.elems[i], out_gravity.covariance.elems[i])
        << "index " << i;
  }
  EXPECT_TRUE(fgb::covarianceIsValid(fgb::covarianceFromMsg(out_fixed.covariance.elems)));
  EXPECT_TRUE(fgb::covarianceIsValid(fgb::covarianceFromMsg(out_gravity.covariance.elems)));

  // 三个姿态互为右乘一个安装旋转的关系：q_X = q_none ⊗ q_IB(X)。
  const Eigen::Quaterniond q_out_none(out_none.pose.orientation.w, out_none.pose.orientation.x,
                                      out_none.pose.orientation.y, out_none.pose.orientation.z);
  const Eigen::Quaterniond q_out_fixed(out_fixed.pose.orientation.w, out_fixed.pose.orientation.x,
                                       out_fixed.pose.orientation.y, out_fixed.pose.orientation.z);
  const Eigen::Quaterniond q_out_gravity(out_gravity.pose.orientation.w,
                                         out_gravity.pose.orientation.x,
                                         out_gravity.pose.orientation.y,
                                         out_gravity.pose.orientation.z);
  expectRotationNear(q_out_fixed.toRotationMatrix(),
                     (q_out_none * q_fixed).normalized().toRotationMatrix(), 1e-15);
  expectRotationNear(q_out_gravity.toRotationMatrix(),
                     (q_out_none * q_gravity).normalized().toRotationMatrix(), 1e-15);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
