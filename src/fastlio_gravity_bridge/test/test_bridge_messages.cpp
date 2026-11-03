// 消息测试：用真实节点类验证等待、输出一致性、非法输入、重复时间戳与重启处理。
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <boost/make_shared.hpp>
#include <gtest/gtest.h>

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "fastlio_gravity_bridge/gravity_bridge.h"

namespace fgb = fastlio_gravity_bridge;

namespace
{

constexpr double kGravity = 9.81;
/// 节点默认的固定安装角（mounting_pitch_deg）：雷达相对机体前倾 20 度。
constexpr double kMountPitchDeg = 20.0;
/// 旧 gravity 模式对照用例用的安装倾角，与默认的 20.0 度刻意区分。
constexpr double kLegacyTiltDeg = 20.12;

Eigen::Matrix3d pitchRotation(double pitch_deg)
{
  return Eigen::AngleAxisd(pitch_deg * M_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

/// 机体水平启动、雷达相对机体前倾 tilt_deg 时，camera_init 下测得的初始化重力。
Eigen::Vector3d tiltedGravity(double tilt_deg = kMountPitchDeg)
{
  return pitchRotation(tilt_deg).transpose() * Eigen::Vector3d(0.0, 0.0, -kGravity);
}

/// fixed 模式在给定安装前倾角下的期望安装旋转：q_IB = R_y(-pitch_deg)。
Eigen::Quaterniond expectedFixedMounting(double pitch_deg)
{
  return Eigen::Quaterniond(
      Eigen::AngleAxisd(-pitch_deg * M_PI / 180.0, Eigen::Vector3d::UnitY()));
}

/// fixed 模式完整安装旋转的真值：R_base<-lidar = R_z(yaw)·R_y(pitch)·R_x(roll)（矩阵相乘）。
Eigen::Matrix3d baseFromLidarRpy(double roll_deg, double pitch_deg, double yaw_deg)
{
  const double deg = M_PI / 180.0;
  return Eigen::AngleAxisd(yaw_deg * deg, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
         Eigen::AngleAxisd(pitch_deg * deg, Eigen::Vector3d::UnitY()).toRotationMatrix() *
         Eigen::AngleAxisd(roll_deg * deg, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

/// 自旋等待，直到条件满足或超时。
template <typename Predicate>
bool spinUntil(Predicate predicate, double timeout_sec)
{
  const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_sec);
  ros::Rate rate(100);
  while (ros::ok() && ros::Time::now() < deadline)
  {
    ros::spinOnce();
    if (predicate()) return true;
    rate.sleep();
  }
  ros::spinOnce();
  return predicate();
}

}  // namespace

class BridgeMessagesTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    static int counter = 0;
    const std::string suffix = std::to_string(counter++);

    ros::NodeHandle nh;
    nh_ = ros::NodeHandlePtr(new ros::NodeHandle());

    config_.odom_in = "/test_bridge/odom" + suffix;
    config_.gravity_in = "/test_bridge/gravity" + suffix;
    config_.odom_out = "/test_bridge/odom_world" + suffix;
    config_.pose_cov_out = "/test_bridge/pose_cov" + suffix;
    config_.velocity_in = "/test_bridge/velocity" + suffix;
    config_.vision_odom_out = "/test_bridge/vision_odom" + suffix;
    config_.publish_static_tf = false;  // 静态 TF 由单独的用例检查

    pub_gravity_ = nh_->advertise<geometry_msgs::Vector3Stamped>(config_.gravity_in, 1, true);
    pub_odom_ = nh_->advertise<nav_msgs::Odometry>(config_.odom_in, 10);
    pub_velocity_ = nh_->advertise<geometry_msgs::Vector3Stamped>(config_.velocity_in, 10);
    sub_odom_ = nh_->subscribe(config_.odom_out, 50, &BridgeMessagesTest::onOdom, this);
    sub_pose_cov_ =
        nh_->subscribe(config_.pose_cov_out, 50, &BridgeMessagesTest::onPoseCov, this);
    sub_vision_odom_ =
        nh_->subscribe(config_.vision_odom_out, 50, &BridgeMessagesTest::onVisionOdom, this);

    odom_count_ = 0;
    pose_cov_count_ = 0;
    vision_odom_count_ = 0;
    vision_odom_stamps_.clear();
  }

  void onVisionOdom(const nav_msgs::Odometry::ConstPtr &msg)
  {
    ++vision_odom_count_;
    last_vision_odom_ = *msg;
    vision_odom_stamps_.push_back(msg->header.stamp);
  }

  /// odom 方式：额外等速度订阅和 vision_odom 发布连上。
  void startOdomModeBridge()
  {
    config_.ev_mode = "odom";
    startBridge();
    ASSERT_TRUE(spinUntil([this] { return pub_velocity_.getNumSubscribers() > 0; }, 10.0));
    ASSERT_TRUE(spinUntil([this] { return sub_vision_odom_.getNumPublishers() > 0; }, 10.0));
  }

  geometry_msgs::Vector3Stamped makeVelocity(double stamp_sec, const Eigen::Vector3d &v_c,
                                             const std::string &frame_id = "camera_init")
  {
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time().fromSec(stamp_sec);
    msg.header.frame_id = frame_id;
    msg.vector.x = v_c.x();
    msg.vector.y = v_c.y();
    msg.vector.z = v_c.z();
    return msg;
  }

  void publishVelocity(double stamp_sec, const Eigen::Vector3d &v_c,
                       const std::string &frame_id = "camera_init")
  {
    pub_velocity_.publish(makeVelocity(stamp_sec, v_c, frame_id));
  }

  /// 不经话题、直接调用回调：两个话题谁先到由用例精确决定。
  void feedOdom(double stamp_sec)
  {
    bridge_->odomCallback(boost::make_shared<nav_msgs::Odometry>(makeOdom(stamp_sec)));
  }

  void feedVelocity(double stamp_sec, const Eigen::Vector3d &v_c)
  {
    bridge_->velocityCallback(
        boost::make_shared<geometry_msgs::Vector3Stamped>(makeVelocity(stamp_sec, v_c)));
  }

  void onOdom(const nav_msgs::Odometry::ConstPtr &msg)
  {
    ++odom_count_;
    last_odom_ = *msg;
  }

  void onPoseCov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
  {
    ++pose_cov_count_;
    last_pose_cov_ = *msg;
  }

  /// 构造桥接节点并等待所有订阅/发布连接建立。
  void startBridge()
  {
    bridge_.reset(new fgb::GravityBridge(*nh_, config_));
    ASSERT_TRUE(spinUntil([this] { return pub_gravity_.getNumSubscribers() > 0; }, 10.0));
    ASSERT_TRUE(spinUntil([this] { return pub_odom_.getNumSubscribers() > 0; }, 10.0));
    ASSERT_TRUE(spinUntil([this] { return sub_odom_.getNumPublishers() > 0; }, 10.0));
    ASSERT_TRUE(spinUntil([this] { return sub_pose_cov_.getNumPublishers() > 0; }, 10.0));
  }

  void publishGravity(const Eigen::Vector3d &gravity, double stamp_sec,
                      const std::string &frame_id = "camera_init")
  {
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time().fromSec(stamp_sec);
    msg.header.frame_id = frame_id;
    msg.vector.x = gravity.x();
    msg.vector.y = gravity.y();
    msg.vector.z = gravity.z();
    pub_gravity_.publish(msg);
  }

  void publishOdom(double stamp_sec, const std::string &frame_id = "camera_init",
                   const std::string &child_frame_id = "body")
  {
    pub_odom_.publish(makeOdom(stamp_sec, frame_id, child_frame_id));
  }

  nav_msgs::Odometry makeOdom(double stamp_sec, const std::string &frame_id = "camera_init",
                              const std::string &child_frame_id = "body")
  {
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time().fromSec(stamp_sec);
    msg.header.frame_id = frame_id;
    msg.child_frame_id = child_frame_id;
    msg.pose.pose.position.x = 1.0;
    msg.pose.pose.position.y = -2.0;
    msg.pose.pose.position.z = 3.0;
    const Eigen::Quaterniond &q_cb = odom_orientation_;
    msg.pose.pose.orientation.w = q_cb.w();
    msg.pose.pose.orientation.x = q_cb.x();
    msg.pose.pose.orientation.y = q_cb.y();
    msg.pose.pose.orientation.z = q_cb.z();
    msg.pose.covariance[0] = 1.0e-2;   // x 位置方差
    msg.pose.covariance[7] = 4.0e-2;   // y 位置方差
    msg.pose.covariance[14] = 9.0e-2;  // z 位置方差
    msg.pose.covariance[21] = 1.0e-5;  // roll 方差
    msg.pose.covariance[28] = 4.0e-5;  // pitch 方差
    msg.pose.covariance[35] = 9.0e-5;  // yaw 方差
    msg.pose.covariance[1] = msg.pose.covariance[6] = 3.0e-3;  // 交叉项
    msg.pose.covariance[3] = msg.pose.covariance[18] = 2.0e-4;
    return msg;
  }

  ros::NodeHandlePtr nh_;
  fgb::GravityBridge::Config config_;
  std::unique_ptr<fgb::GravityBridge> bridge_;
  /// publishOdom 使用的姿态，默认给一个非单位四元数以便区分变换前后。
  Eigen::Quaterniond odom_orientation_ =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()));

  ros::Publisher pub_gravity_;
  ros::Publisher pub_odom_;
  ros::Publisher pub_velocity_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_pose_cov_;
  ros::Subscriber sub_vision_odom_;

  int odom_count_ = 0;
  int pose_cov_count_ = 0;
  int vision_odom_count_ = 0;
  nav_msgs::Odometry last_odom_;
  geometry_msgs::PoseWithCovarianceStamped last_pose_cov_;
  nav_msgs::Odometry last_vision_odom_;
  std::vector<ros::Time> vision_odom_stamps_;
};

/// 未收到有效初始化重力前只等待，不输出未对齐消息。
TEST_F(BridgeMessagesTest, WaitsForGravityBeforePublishing)
{
  startBridge();
  EXPECT_EQ(bridge_->state(), fgb::GravityBridge::State::kWaitingGravity);

  publishOdom(100.0);
  spinUntil([this] { return odom_count_ > 0; }, 0.3);

  EXPECT_EQ(odom_count_, 0);
  EXPECT_EQ(pose_cov_count_, 0);
  EXPECT_EQ(bridge_->publishedCount(), 0u);
}

/// 初始化重力坐标系不符时继续等待，不会锁定错误的对齐旋转。
TEST_F(BridgeMessagesTest, RejectsGravityWithWrongFrame)
{
  startBridge();

  publishGravity(tiltedGravity(), 55.0, "odom");  // 坐标系错误
  spinUntil([this] { return bridge_->hasAlignment(); }, 0.3);
  EXPECT_FALSE(bridge_->hasAlignment());
  EXPECT_EQ(bridge_->state(), fgb::GravityBridge::State::kWaitingGravity);

  publishOdom(100.5);
  spinUntil([this] { return odom_count_ > 0; }, 0.3);
  EXPECT_EQ(odom_count_, 0);

  publishGravity(tiltedGravity(), 56.0);  // 正确坐标系
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));
  publishOdom(100.5);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 0; }, 5.0));
  EXPECT_EQ(odom_count_, 1);
}

/// 默认固定安装补偿：初始化时机体水平，则此后输出姿态表示机体 —— 与初始化时刻相同的
/// 姿态应当读成水平（pitch ≈ 0），而不是雷达的 20 度前倾。
TEST_F(BridgeMessagesTest, MountingMakesAttitudeLevelAtInit)
{
  startBridge();
  // 雷达前倾 20 度安装、初始化时机体水平 -> 初始化重力带着这个倾角
  publishGravity(tiltedGravity(), 400.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));

  const fgb::AlignmentTransform &alignment = bridge_->alignment();
  // 对齐旋转本身就是那个安装倾角：不做补偿时初始化时刻的姿态读数即 20 度前倾
  EXPECT_NEAR(alignment.q.angularDistance(Eigen::Quaterniond::Identity()) * 180.0 / M_PI,
              kMountPitchDeg, 0.5);

  // 与初始化时刻相同的姿态（q_CI = I）
  odom_orientation_ = Eigen::Quaterniond::Identity();
  publishOdom(400.0);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 0; }, 5.0));

  const Eigen::Quaterniond q_out(last_odom_.pose.pose.orientation.w,
                                 last_odom_.pose.pose.orientation.x,
                                 last_odom_.pose.pose.orientation.y,
                                 last_odom_.pose.pose.orientation.z);
  EXPECT_LT((q_out.toRotationMatrix() - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), 1e-12);

  // 安装旋转恰好是 20 度前倾的逆（默认 fixed 模式，与初始化重力无关）
  const Eigen::Quaterniond &q_ib = bridge_->mounting();
  EXPECT_NEAR(q_ib.norm(), 1.0, 1e-15);
  EXPECT_LT((q_ib.toRotationMatrix() - expectedFixedMounting(kMountPitchDeg).toRotationMatrix())
                .cwiseAbs().maxCoeff(),
            1e-15);
}

/// mounting_source:=none 时恢复旧行为：输出仍是雷达 IMU 姿态，child frame 也应是 body。
TEST_F(BridgeMessagesTest, MountingSourceNoneKeepsImuAttitude)
{
  config_.mounting_source = "none";
  config_.output_child_frame = "body";
  startBridge();
  publishGravity(tiltedGravity(), 410.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));

  EXPECT_LT(bridge_->mounting().angularDistance(Eigen::Quaterniond::Identity()), 1e-15);

  odom_orientation_ = Eigen::Quaterniond::Identity();
  publishOdom(410.0);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 0; }, 5.0));

  EXPECT_EQ(last_odom_.child_frame_id, "body");
  // 输出姿态 = q_a（仍是 20 度前倾），没有被安装旋转抵消
  const Eigen::Quaterniond q_out(last_odom_.pose.pose.orientation.w,
                                 last_odom_.pose.pose.orientation.x,
                                 last_odom_.pose.pose.orientation.y,
                                 last_odom_.pose.pose.orientation.z);
  const Eigen::Quaterniond &q_a = bridge_->alignment().q;
  EXPECT_LT((q_out.toRotationMatrix() - q_a.toRotationMatrix()).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_GT((q_out.toRotationMatrix() - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), 0.1);
}

/// 默认配置是固定安装补偿：安装旋转只由 mounting_pitch_deg（默认 20 度）决定，
/// 启动时即已确定、与初始化重力无关；初始化时机体不水平时输出保留真实倾角。
TEST_F(BridgeMessagesTest, DefaultMountingIsFixedAndDoesNotDependOnGravity)
{
  startBridge();  // config_ 用默认值：mounting_source=fixed，mounting_pitch_deg=20.0
  EXPECT_EQ(config_.mounting_source, "fixed");
  EXPECT_DOUBLE_EQ(config_.mounting_pitch_deg, kMountPitchDeg);

  // 还没收到初始化重力，安装旋转已经就位（不依赖重力）
  EXPECT_LT((bridge_->mounting().toRotationMatrix() -
             expectedFixedMounting(kMountPitchDeg).toRotationMatrix())
                .cwiseAbs().maxCoeff(),
            1e-15);
  EXPECT_FALSE(bridge_->hasAlignment());

  // 初始化时雷达系相对世界竖直倾斜 30 度 -> 机体本身前倾 10 度
  publishGravity(tiltedGravity(30.0), 500.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));

  // 安装旋转不随初始化重力改变
  EXPECT_LT((bridge_->mounting().toRotationMatrix() -
             expectedFixedMounting(kMountPitchDeg).toRotationMatrix())
                .cwiseAbs().maxCoeff(),
            1e-15);

  // 与初始化时刻相同的 /Odometry 姿态（q_CI = I）：输出应是真实的 10 度前倾，
  // 而不是像旧 gravity 模式那样被抵消成水平。
  odom_orientation_ = Eigen::Quaterniond::Identity();
  publishOdom(500.0);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 0; }, 5.0));

  const Eigen::Quaterniond q_out(last_odom_.pose.pose.orientation.w,
                                 last_odom_.pose.pose.orientation.x,
                                 last_odom_.pose.pose.orientation.y,
                                 last_odom_.pose.pose.orientation.z);
  const Eigen::Quaterniond q_expected(
      Eigen::AngleAxisd(10.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()));
  EXPECT_LT((q_out.toRotationMatrix() - q_expected.toRotationMatrix()).cwiseAbs().maxCoeff(),
            1e-12);
  // 确实保留了倾角：不是单位姿态
  EXPECT_GT((q_out.toRotationMatrix() - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), 0.1);
}

/// 旧模式兼容：mounting_source:=gravity 仍由初始化重力推导 q_IB = q_a⁻¹，
/// 初始化时机体水平时输出水平（与 fixed 在 20 度安装下的结果一致）。
TEST_F(BridgeMessagesTest, MountingSourceGravityKeepsLegacyBehavior)
{
  config_.mounting_source = "gravity";
  startBridge();
  // 未锁定对齐旋转前不做安装补偿
  EXPECT_LT(bridge_->mounting().angularDistance(Eigen::Quaterniond::Identity()), 1e-15);

  publishGravity(tiltedGravity(kLegacyTiltDeg), 510.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));

  const Eigen::Quaterniond &q_ib = bridge_->mounting();
  const Eigen::Quaterniond &q_a = bridge_->alignment().q;
  EXPECT_LT((q_ib.toRotationMatrix() - q_a.conjugate().toRotationMatrix()).cwiseAbs().maxCoeff(),
            1e-15);

  odom_orientation_ = Eigen::Quaterniond::Identity();
  publishOdom(510.0);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 0; }, 5.0));

  const Eigen::Quaterniond q_out(last_odom_.pose.pose.orientation.w,
                                 last_odom_.pose.pose.orientation.x,
                                 last_odom_.pose.pose.orientation.y,
                                 last_odom_.pose.pose.orientation.z);
  EXPECT_LT((q_out.toRotationMatrix() - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), 1e-12);
}

/// 旧 gravity 模式的已知前提：初始化时机体本身倾斜时，初始倾角会被当作安装角一并
/// 抵消（这正是默认改用固定安装补偿的原因）。fixed 模式不会这样，见上一个用例。
TEST_F(BridgeMessagesTest, LegacyGravityModeCancelsTiltedStart)
{
  config_.mounting_source = "gravity";
  startBridge();

  // 初始化时雷达系相对竖直 30 度、安装 20 度 -> 机体真实前倾 10 度
  publishGravity(tiltedGravity(30.0), 520.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));

  odom_orientation_ = Eigen::Quaterniond::Identity();
  publishOdom(520.0);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 0; }, 5.0));

  const Eigen::Quaterniond q_out(last_odom_.pose.pose.orientation.w,
                                 last_odom_.pose.pose.orientation.x,
                                 last_odom_.pose.pose.orientation.y,
                                 last_odom_.pose.pose.orientation.z);
  // 10 度的真实倾角被归零（旧行为）
  EXPECT_LT((q_out.toRotationMatrix() - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), 1e-12);
}

/// 非法安装配置（未知来源 / 非有限角度）下不建立订阅与发布，也不输出任何姿态：
/// 明确报错而不是静默兜底成"不补偿"。
TEST_F(BridgeMessagesTest, InvalidMountingConfigurationStopsTheBridge)
{
  config_.mounting_source = "bogus";
  bridge_.reset(new fgb::GravityBridge(*nh_, config_));
  EXPECT_EQ(bridge_->state(), fgb::GravityBridge::State::kFatal);
  EXPECT_FALSE(bridge_->fatalReason().empty());
  EXPECT_FALSE(bridge_->hasAlignment());

  publishGravity(tiltedGravity(), 530.0);
  publishOdom(530.0);
  spinUntil([this] { return odom_count_ > 0; }, 0.3);
  EXPECT_EQ(odom_count_, 0);
  EXPECT_EQ(pose_cov_count_, 0);
  EXPECT_EQ(bridge_->publishedCount(), 0u);

  // 非有限安装角同样拒绝
  config_.mounting_source = "fixed";
  config_.mounting_pitch_deg = std::numeric_limits<double>::quiet_NaN();
  bridge_.reset(new fgb::GravityBridge(*nh_, config_));
  EXPECT_EQ(bridge_->state(), fgb::GravityBridge::State::kFatal);
  EXPECT_FALSE(bridge_->fatalReason().empty());

  // 非有限的安装横滚 / 偏航也拒绝
  config_.mounting_pitch_deg = kMountPitchDeg;
  config_.mounting_roll_deg = std::numeric_limits<double>::quiet_NaN();
  bridge_.reset(new fgb::GravityBridge(*nh_, config_));
  EXPECT_EQ(bridge_->state(), fgb::GravityBridge::State::kFatal);
  EXPECT_NE(bridge_->fatalReason().find("mounting_roll_deg"), std::string::npos)
      << bridge_->fatalReason();

  config_.mounting_roll_deg = 0.0;
  config_.mounting_yaw_deg = std::numeric_limits<double>::infinity();
  bridge_.reset(new fgb::GravityBridge(*nh_, config_));
  EXPECT_EQ(bridge_->state(), fgb::GravityBridge::State::kFatal);
  EXPECT_FALSE(bridge_->fatalReason().empty());
}

// ---------------------------------------------------------------------------
// 参数校验：默认 fixed 20 度、旧模式仍可用、非法配置必须让启动失败。
// ---------------------------------------------------------------------------
namespace
{

fgb::GravityBridge::Config loadConfigFrom(const std::string &ns)
{
  ros::NodeHandle pnh(ns);
  return fgb::GravityBridge::loadConfig(pnh);
}

}  // namespace

TEST(BridgeConfigValidation, DefaultsToFixedTwentyDegrees)
{
  const fgb::GravityBridge::Config config = loadConfigFrom("/test_bridge_config_default");
  EXPECT_TRUE(config.valid) << config.error;
  EXPECT_EQ(config.mounting_source, "fixed");
  EXPECT_DOUBLE_EQ(config.mounting_pitch_deg, kMountPitchDeg);
  // 默认安装横滚、偏航为 0：保持纯俯仰的旧行为
  EXPECT_DOUBLE_EQ(config.mounting_roll_deg, 0.0);
  EXPECT_DOUBLE_EQ(config.mounting_yaw_deg, 0.0);
}

TEST(BridgeConfigValidation, ReadsExplicitMountingParameters)
{
  ros::param::set("/test_bridge_config_explicit/mounting_source", std::string("fixed"));
  ros::param::set("/test_bridge_config_explicit/mounting_roll_deg", 0.35);
  ros::param::set("/test_bridge_config_explicit/mounting_pitch_deg", 17.5);
  ros::param::set("/test_bridge_config_explicit/mounting_yaw_deg", -0.8);

  const fgb::GravityBridge::Config config = loadConfigFrom("/test_bridge_config_explicit");
  EXPECT_TRUE(config.valid) << config.error;
  EXPECT_EQ(config.mounting_source, "fixed");
  EXPECT_DOUBLE_EQ(config.mounting_roll_deg, 0.35);
  EXPECT_DOUBLE_EQ(config.mounting_pitch_deg, 17.5);
  EXPECT_DOUBLE_EQ(config.mounting_yaw_deg, -0.8);
}

TEST(BridgeConfigValidation, KeepsLegacySourcesWorking)
{
  for (const std::string source : {"gravity", "none"})
  {
    const std::string ns = "/test_bridge_config_legacy_" + source;
    ros::param::set(ns + "/mounting_source", source);

    const fgb::GravityBridge::Config config = loadConfigFrom(ns);
    EXPECT_TRUE(config.valid) << config.error;
    EXPECT_EQ(config.mounting_source, source);
    // mounting_pitch_deg 只在 fixed 模式生效，但默认值仍然是 20 度
    EXPECT_DOUBLE_EQ(config.mounting_pitch_deg, kMountPitchDeg);
  }
}

TEST(BridgeConfigValidation, RejectsUnknownSource)
{
  ros::param::set("/test_bridge_config_bad_source/mounting_source", std::string("gps"));
  const fgb::GravityBridge::Config config = loadConfigFrom("/test_bridge_config_bad_source");
  EXPECT_FALSE(config.valid);
  EXPECT_FALSE(config.error.empty());
  EXPECT_NE(config.error.find("gps"), std::string::npos) << config.error;
}

TEST(BridgeConfigValidation, RejectsNonFiniteMountingPitch)
{
  ros::param::set("/test_bridge_config_nan_pitch/mounting_source", std::string("fixed"));
  ros::param::set("/test_bridge_config_nan_pitch/mounting_pitch_deg",
                  std::numeric_limits<double>::quiet_NaN());
  const fgb::GravityBridge::Config config = loadConfigFrom("/test_bridge_config_nan_pitch");
  EXPECT_FALSE(config.valid);
  EXPECT_FALSE(config.error.empty());
}

TEST(BridgeConfigValidation, RejectsNonFiniteMountingRollOrYaw)
{
  for (const std::string name : {"mounting_roll_deg", "mounting_yaw_deg"})
  {
    const std::string ns = "/test_bridge_config_nan_" + name;
    ros::param::set(ns + "/mounting_source", std::string("fixed"));
    ros::param::set(ns + "/" + name, std::numeric_limits<double>::quiet_NaN());
    const fgb::GravityBridge::Config config = loadConfigFrom(ns);
    EXPECT_FALSE(config.valid) << name;
    EXPECT_NE(config.error.find(name), std::string::npos) << config.error;
  }
}

/// latched 重力：桥接节点晚于发布者启动（订阅发生在发布之后）也能拿到初始化重力。
TEST_F(BridgeMessagesTest, ReceivesLatchedGravityAfterLateSubscription)
{
  // 先发布（此时还没有订阅者），再构造桥接节点。
  publishGravity(tiltedGravity(), 50.0);
  spinUntil([this] { return false; }, 0.2);

  startBridge();
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));
  EXPECT_EQ(bridge_->state(), fgb::GravityBridge::State::kAligned);

  publishOdom(101.0);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 0; }, 5.0));
  EXPECT_EQ(odom_count_, 1);
  EXPECT_EQ(pose_cov_count_, 1);
}

/// 两路输出：位姿、协方差、时间戳一致；坐标系与时间戳按约定保留。
TEST_F(BridgeMessagesTest, OutputsSharePoseCovarianceAndTimestamp)
{
  startBridge();
  publishGravity(tiltedGravity(), 60.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));

  publishOdom(102.5);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 0 && pose_cov_count_ > 0; }, 5.0));

  const fgb::AlignmentTransform &alignment = bridge_->alignment();

  // 时间戳保留输入测量时间，父坐标系为 drone_odom，子坐标系为 drone_base（姿态已是机体）。
  EXPECT_DOUBLE_EQ(last_odom_.header.stamp.toSec(), 102.5);
  EXPECT_EQ(last_odom_.header.frame_id, "drone_odom");
  EXPECT_EQ(last_odom_.child_frame_id, "drone_base");
  EXPECT_EQ(last_pose_cov_.header.frame_id, "drone_odom");
  EXPECT_DOUBLE_EQ(last_pose_cov_.header.stamp.toSec(), 102.5);

  // 位姿：p_W = R_a p_C，q_out = q_a ⊗ q_CB ⊗ q_IB。
  const Eigen::Vector3d p_world = alignment.R * Eigen::Vector3d(1.0, -2.0, 3.0);
  EXPECT_NEAR(last_odom_.pose.pose.position.x, p_world.x(), 1e-12);
  EXPECT_NEAR(last_odom_.pose.pose.position.y, p_world.y(), 1e-12);
  EXPECT_NEAR(last_odom_.pose.pose.position.z, p_world.z(), 1e-12);
  const Eigen::Quaterniond q_cb(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()));
  const Eigen::Quaterniond q_expected = alignment.q * q_cb * bridge_->mounting();
  const Eigen::Quaterniond q_actual(last_odom_.pose.pose.orientation.w,
                                    last_odom_.pose.pose.orientation.x,
                                    last_odom_.pose.pose.orientation.y,
                                    last_odom_.pose.pose.orientation.z);
  EXPECT_LT((q_actual.toRotationMatrix() - q_expected.toRotationMatrix()).cwiseAbs().maxCoeff(),
            1e-12);

  // 对齐后重力沿 drone_odom 的 -Z。
  const Eigen::Vector3d g_world = alignment.R * tiltedGravity().normalized();
  EXPECT_LT(std::abs(g_world.x()), 1e-12);
  EXPECT_LT(std::abs(g_world.y()), 1e-12);
  EXPECT_NEAR(g_world.z(), -1.0, 1e-12);

  // 两路输出的位姿与协方差逐位一致。
  EXPECT_DOUBLE_EQ(last_odom_.pose.pose.position.x, last_pose_cov_.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(last_odom_.pose.pose.orientation.w, last_pose_cov_.pose.pose.orientation.w);
  for (int i = 0; i < 36; ++i)
  {
    EXPECT_DOUBLE_EQ(last_odom_.pose.covariance[i], last_pose_cov_.pose.covariance[i])
        << "covariance index " << i;
  }

  // twist 及其协方差原样保留（当前 FAST-LIO2 未提供有效速度，输入为零）。
  EXPECT_DOUBLE_EQ(last_odom_.twist.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(last_odom_.twist.twist.angular.x, 0.0);
  for (int i = 0; i < 36; ++i)
  {
    EXPECT_DOUBLE_EQ(last_odom_.twist.covariance[i], 0.0);
  }
}

/// 非法输入一律拒绝发布：坐标系错误、非有限位姿、无效四元数、非法协方差。
TEST_F(BridgeMessagesTest, RejectsInvalidInputs)
{
  startBridge();
  publishGravity(tiltedGravity(), 70.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));

  publishOdom(103.0, "odom", "body");  // 输入坐标系错误
  spinUntil([this] { return odom_count_ > 0; }, 0.3);
  EXPECT_EQ(odom_count_, 0);

  publishOdom(103.0, "camera_init", "drone_base");  // 输入 child_frame 错误（应为 body）
  spinUntil([this] { return odom_count_ > 0; }, 0.3);
  EXPECT_EQ(odom_count_, 0);

  {
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time().fromSec(103.0);
    msg.header.frame_id = "camera_init";
    msg.child_frame_id = "body";
    msg.pose.pose.orientation.w = 1.0;
    msg.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
    pub_odom_.publish(msg);
  }
  spinUntil([this] { return odom_count_ > 0; }, 0.3);
  EXPECT_EQ(odom_count_, 0);

  {
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time().fromSec(103.0);
    msg.header.frame_id = "camera_init";
    msg.child_frame_id = "body";
    msg.pose.pose.orientation.w = 0.0;  // 无效四元数
    pub_odom_.publish(msg);
  }
  spinUntil([this] { return odom_count_ > 0; }, 0.3);
  EXPECT_EQ(odom_count_, 0);

  {
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time().fromSec(103.0);
    msg.header.frame_id = "camera_init";
    msg.child_frame_id = "body";
    msg.pose.pose.orientation.w = 1.0;
    msg.pose.covariance[21] = -1.0;  // 负方差
    pub_odom_.publish(msg);
  }
  spinUntil([this] { return odom_count_ > 0; }, 0.3);
  EXPECT_EQ(odom_count_, 0);

  // 同一时间戳的合法消息仍应正常输出（前面几条都被拒绝，未占用时间戳）。
  publishOdom(103.0);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 0; }, 5.0));
  EXPECT_EQ(odom_count_, 1);
}

/// 早于本次初始化的 odom 丢弃（不算时间回退）：历史帧晚到时不会用本轮旋转处理。
TEST_F(BridgeMessagesTest, DropsOdomEarlierThanGravityInit)
{
  startBridge();
  publishGravity(tiltedGravity(), 200.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));

  // 首条输入就早于初始化时间（此前没有已发布帧，旧实现会直接输出）。
  publishOdom(199.0);
  spinUntil([this] { return odom_count_ > 0; }, 0.3);
  EXPECT_EQ(odom_count_, 0);
  EXPECT_EQ(pose_cov_count_, 0);
  EXPECT_EQ(bridge_->state(), fgb::GravityBridge::State::kAligned);  // 只是丢弃，不是致命

  // 与初始化同一时刻的帧允许通过。
  publishOdom(200.0);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 0; }, 5.0));
  EXPECT_EQ(odom_count_, 1);

  // 初始化之后、但早于已发布帧的输入：按“时间回退”处理（停止输出）。
  publishOdom(205.0);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 1; }, 5.0));
  publishOdom(202.0);
  ASSERT_TRUE(spinUntil(
      [this] { return bridge_->state() == fgb::GravityBridge::State::kFatal; }, 5.0));
}

/// 初始化前只等待：这段时间到达的 odom 不应被缓存后补发。
TEST_F(BridgeMessagesTest, DoesNotReplayOdomBufferedBeforeInit)
{
  startBridge();
  for (double stamp = 300.0; stamp < 300.5; stamp += 0.1)
  {
    publishOdom(stamp);
  }
  spinUntil([this] { return odom_count_ > 0; }, 0.3);
  EXPECT_EQ(odom_count_, 0);

  publishGravity(tiltedGravity(), 301.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));
  spinUntil([this] { return odom_count_ > 0; }, 0.5);
  EXPECT_EQ(odom_count_, 0);  // 初始化之前的历史帧不会补发

  publishOdom(302.0);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 0; }, 5.0));
  EXPECT_EQ(odom_count_, 1);
}

/// 重复时间戳丢弃，后续正常时间戳继续输出。
TEST_F(BridgeMessagesTest, DropsDuplicateTimestamps)
{
  startBridge();
  publishGravity(tiltedGravity(), 80.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));

  publishOdom(104.0);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 0; }, 5.0));
  EXPECT_EQ(odom_count_, 1);

  publishOdom(104.0);  // 重复时间戳
  spinUntil([this] { return odom_count_ > 1; }, 0.3);
  EXPECT_EQ(odom_count_, 1);
  EXPECT_EQ(bridge_->state(), fgb::GravityBridge::State::kAligned);

  publishOdom(104.1);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 1; }, 5.0));
  EXPECT_EQ(odom_count_, 2);
}

/// 时间回退：停止输出并要求重启，之后的新消息也不再输出。
TEST_F(BridgeMessagesTest, StopsOnTimestampRegression)
{
  startBridge();
  publishGravity(tiltedGravity(), 90.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));

  publishOdom(105.0);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 0; }, 5.0));

  publishOdom(104.0);  // 时间回退
  ASSERT_TRUE(spinUntil(
      [this] { return bridge_->state() == fgb::GravityBridge::State::kFatal; }, 5.0));
  const int count_after_fatal = odom_count_;

  publishOdom(106.0);
  spinUntil([this, count_after_fatal] { return odom_count_ > count_after_fatal; }, 0.3);
  EXPECT_EQ(odom_count_, count_after_fatal);
  EXPECT_EQ(pose_cov_count_, count_after_fatal);
}

/// 运行中出现另一轮初始化：停止输出。
TEST_F(BridgeMessagesTest, StopsOnSecondGravityInitialization)
{
  startBridge();
  publishGravity(tiltedGravity(), 95.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));

  publishOdom(107.0);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 0; }, 5.0));
  const int count_before = odom_count_;

  publishGravity(tiltedGravity() * 1.001, 96.0);  // 另一轮初始化
  ASSERT_TRUE(spinUntil(
      [this] { return bridge_->state() == fgb::GravityBridge::State::kFatal; }, 5.0));

  publishOdom(108.0);
  spinUntil([this, count_before] { return odom_count_ > count_before; }, 0.3);
  EXPECT_EQ(odom_count_, count_before);
  EXPECT_FALSE(bridge_->fatalReason().empty());
}

/// 默认发布 drone_odom -> camera_init 静态 TF（latched，晚订阅也能查询到）。
TEST_F(BridgeMessagesTest, PublishesStaticTransform)
{
  config_.publish_static_tf = true;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  startBridge();
  publishGravity(tiltedGravity(), 97.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));

  geometry_msgs::TransformStamped transform;
  ASSERT_TRUE(spinUntil(
      [&] {
        try
        {
          transform = tf_buffer.lookupTransform("drone_odom", "camera_init", ros::Time(0));
          return true;
        }
        catch (const tf2::TransformException &)
        {
          return false;
        }
      },
      5.0));

  EXPECT_EQ(transform.header.frame_id, "drone_odom");
  EXPECT_EQ(transform.child_frame_id, "camera_init");
  EXPECT_NEAR(transform.transform.translation.x, 0.0, 1e-15);
  EXPECT_NEAR(transform.transform.translation.y, 0.0, 1e-15);
  EXPECT_NEAR(transform.transform.translation.z, 0.0, 1e-15);
  const Eigen::Quaterniond q(transform.transform.rotation.w, transform.transform.rotation.x,
                             transform.transform.rotation.y, transform.transform.rotation.z);
  EXPECT_LT((q.toRotationMatrix() - bridge_->alignment().R).cwiseAbs().maxCoeff(), 1e-12);

  // 同时发布 body -> drone_base 的安装旋转，保证 TF 树与 odom 消息自洽
  geometry_msgs::TransformStamped mount;
  ASSERT_TRUE(spinUntil(
      [&] {
        try
        {
          mount = tf_buffer.lookupTransform("body", "drone_base", ros::Time(0));
          return true;
        }
        catch (const tf2::TransformException &)
        {
          return false;
        }
      },
      5.0));
  EXPECT_EQ(mount.header.frame_id, "body");
  EXPECT_EQ(mount.child_frame_id, "drone_base");
  EXPECT_NEAR(mount.transform.translation.x, 0.0, 1e-15);  // 原点不动，杆臂不补偿
  EXPECT_NEAR(mount.transform.translation.y, 0.0, 1e-15);
  EXPECT_NEAR(mount.transform.translation.z, 0.0, 1e-15);
  const Eigen::Quaterniond q_mount(mount.transform.rotation.w, mount.transform.rotation.x,
                                   mount.transform.rotation.y, mount.transform.rotation.z);
  const Eigen::Quaterniond &q_ib = bridge_->mounting();
  EXPECT_LT((q_mount.toRotationMatrix() - q_ib.toRotationMatrix()).cwiseAbs().maxCoeff(), 1e-12);

  // 完整的 drone_odom -> camera_init -> body -> drone_base 链需要 FAST-LIO 提供 camera_init -> body
  // 这一段，单测里没有该发布者，因此这里只验证两段静态 TF 的复合：
  // drone_odom -> drone_base 的旋转应等于 q_a ⊗ q_ib（回放验证里再查完整链路）。
  const Eigen::Quaterniond q_composed = (bridge_->alignment().q * q_ib).normalized();
  const Eigen::Quaterniond q_static_chain =
      (q * q_mount).normalized();
  EXPECT_LT((q_static_chain.toRotationMatrix() - q_composed.toRotationMatrix())
                .cwiseAbs().maxCoeff(),
            1e-12);
}

/// fixed 模式的完整安装旋转（roll / pitch / yaw）：安装旋转、body -> drone_base 静态 TF、
/// 输出姿态 q_a ⊗ q_cb ⊗ q_IB 三者一致。坐标系名与其它用例分开，避免静态 TF 互相干扰。
TEST_F(BridgeMessagesTest, FixedMountingUsesFullRpy)
{
  config_.mounting_roll_deg = 0.35;
  config_.mounting_pitch_deg = 20.12;
  config_.mounting_yaw_deg = -0.80;
  config_.world_frame = "world_rpy";
  config_.input_frame = "camera_init_rpy";
  config_.input_child_frame = "body_rpy";
  config_.output_child_frame = "base_link_rpy";
  config_.publish_static_tf = true;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  startBridge();
  // 构造时即按三个角度确定安装旋转：q_IB = (R_base<-lidar)⁻¹，与初始化重力无关
  const Eigen::Matrix3d R_base_lidar = baseFromLidarRpy(0.35, 20.12, -0.80);
  EXPECT_LT((bridge_->mounting().toRotationMatrix() - R_base_lidar.transpose())
                .cwiseAbs().maxCoeff(),
            1e-14);

  // 机体水平启动：初始化重力是世界重力在雷达系里的表达
  publishGravity(R_base_lidar.transpose() * Eigen::Vector3d(0.0, 0.0, -kGravity), 700.0,
                 "camera_init_rpy");
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));

  publishOdom(700.5, "camera_init_rpy", "body_rpy");
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 0; }, 5.0));
  EXPECT_EQ(last_odom_.header.frame_id, "world_rpy");
  EXPECT_EQ(last_odom_.child_frame_id, "base_link_rpy");
  const Eigen::Quaterniond q_out(last_odom_.pose.pose.orientation.w,
                                 last_odom_.pose.pose.orientation.x,
                                 last_odom_.pose.pose.orientation.y,
                                 last_odom_.pose.pose.orientation.z);
  const Eigen::Quaterniond q_expected =
      (bridge_->alignment().q * odom_orientation_ * bridge_->mounting()).normalized();
  EXPECT_LT((q_out.toRotationMatrix() - q_expected.toRotationMatrix()).cwiseAbs().maxCoeff(),
            1e-12);

  // 与初始化时刻相同的雷达姿态（q_CI = I）：机体水平，输出只剩绕竖直轴的偏航基准
  odom_orientation_ = Eigen::Quaterniond::Identity();
  publishOdom(701.0, "camera_init_rpy", "body_rpy");
  ASSERT_TRUE(spinUntil([this] { return odom_count_ > 1; }, 5.0));
  const Eigen::Quaterniond q_level(last_odom_.pose.pose.orientation.w,
                                   last_odom_.pose.pose.orientation.x,
                                   last_odom_.pose.pose.orientation.y,
                                   last_odom_.pose.pose.orientation.z);
  EXPECT_NEAR((q_level * Eigen::Vector3d::UnitZ()).z(), 1.0, 1e-12);

  // body -> drone_base 静态 TF 就是同一个安装旋转，平移为零
  geometry_msgs::TransformStamped mount;
  ASSERT_TRUE(spinUntil(
      [&] {
        try
        {
          mount = tf_buffer.lookupTransform("body_rpy", "base_link_rpy", ros::Time(0));
          return true;
        }
        catch (const tf2::TransformException &)
        {
          return false;
        }
      },
      5.0));
  EXPECT_NEAR(mount.transform.translation.x, 0.0, 1e-15);
  EXPECT_NEAR(mount.transform.translation.y, 0.0, 1e-15);
  EXPECT_NEAR(mount.transform.translation.z, 0.0, 1e-15);
  const Eigen::Quaterniond q_mount(mount.transform.rotation.w, mount.transform.rotation.x,
                                   mount.transform.rotation.y, mount.transform.rotation.z);
  EXPECT_LT((q_mount.toRotationMatrix() - bridge_->mounting().toRotationMatrix())
                .cwiseAbs().maxCoeff(),
            1e-12);
}

// ---------------------------------------------------------------------------
// 送飞控方式 ev_mode：pose 只发位姿；odom 把位姿和同一时间戳的速度配成一条 Odometry。
// ---------------------------------------------------------------------------

TEST(BridgeConfigValidation, EvModeDefaultsToPose)
{
  const fgb::GravityBridge::Config config = loadConfigFrom("/test_bridge_config_ev_default");
  EXPECT_TRUE(config.valid) << config.error;
  EXPECT_EQ(config.ev_mode, "pose");
  EXPECT_EQ(config.velocity_in, "/fastlio/velocity");
  EXPECT_EQ(config.mavros_odom_topic, "/mavros/odometry/out");
  EXPECT_EQ(config.mavros_odom_frame, "px4_odom");
  EXPECT_DOUBLE_EQ(config.odom_velocity_std, 0.0);
}

TEST(BridgeConfigValidation, ReadsOdomModeParameters)
{
  const std::string ns = "/test_bridge_config_ev_odom";
  ros::param::set(ns + "/ev_mode", std::string("odom"));
  ros::param::set(ns + "/odom_velocity_std", 0.15);
  const fgb::GravityBridge::Config config = loadConfigFrom(ns);
  EXPECT_TRUE(config.valid) << config.error;
  EXPECT_EQ(config.ev_mode, "odom");
  EXPECT_DOUBLE_EQ(config.odom_velocity_std, 0.15);
}

TEST(BridgeConfigValidation, RejectsUnknownEvModeOrBadVelocityStd)
{
  ros::param::set("/test_bridge_config_ev_bad/ev_mode", std::string("vision_speed"));
  fgb::GravityBridge::Config config = loadConfigFrom("/test_bridge_config_ev_bad");
  EXPECT_FALSE(config.valid);
  EXPECT_NE(config.error.find("vision_speed"), std::string::npos) << config.error;

  for (const double bad : {-0.1, std::numeric_limits<double>::quiet_NaN()})
  {
    const std::string ns = "/test_bridge_config_ev_std_" + std::to_string(bad < 0.0 ? 0 : 1);
    ros::param::set(ns + "/ev_mode", std::string("odom"));
    ros::param::set(ns + "/odom_velocity_std", bad);
    config = loadConfigFrom(ns);
    EXPECT_FALSE(config.valid) << bad;
    EXPECT_NE(config.error.find("odom_velocity_std"), std::string::npos) << config.error;
  }
}

/// 构造时 ev_mode 非法：直接停住，不建立任何订阅/发布。
TEST_F(BridgeMessagesTest, InvalidEvModeStopsTheBridge)
{
  config_.ev_mode = "both";
  bridge_.reset(new fgb::GravityBridge(*nh_, config_));
  EXPECT_EQ(bridge_->state(), fgb::GravityBridge::State::kFatal);
  EXPECT_NE(bridge_->fatalReason().find("ev_mode"), std::string::npos) << bridge_->fatalReason();
  EXPECT_EQ(pub_odom_.getNumSubscribers(), 0u);
}

/// pose 方式不订阅速度、不发 vision_odom，行为与加这个参数之前一致。
TEST_F(BridgeMessagesTest, PoseModeSendsNoVelocity)
{
  startBridge();
  EXPECT_EQ(pub_velocity_.getNumSubscribers(), 0u);
  publishGravity(tiltedGravity(), 900.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));
  publishOdom(900.5);
  publishVelocity(900.5, Eigen::Vector3d(0.2, 0.0, 0.0));
  ASSERT_TRUE(spinUntil([this] { return pose_cov_count_ > 0; }, 5.0));
  spinUntil([this] { return vision_odom_count_ > 0; }, 0.3);
  EXPECT_EQ(vision_odom_count_, 0);
  EXPECT_EQ(bridge_->visionOdomCount(), 0u);
}

/// odom 方式：位姿与 drone_odom 输出一致，速度按真实物理关系还原成机体系速度；
/// 帧名按 MAVROS odometry 插件的要求（父 px4_odom、子 drone_base），角速度未知填 NaN。
TEST_F(BridgeMessagesTest, OdomModePairsPoseWithBodyVelocity)
{
  config_.odom_velocity_std = 0.2;
  startOdomModeBridge();
  // 机体水平启动、雷达按默认 20 度前倾安装
  const Eigen::Matrix3d R_base_lidar = pitchRotation(kMountPitchDeg);
  publishGravity(tiltedGravity(), 1000.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));

  // 之后机体偏航 90 度、抬头 5 度，沿机体系 (前 0.4, 左 0.1, 上 0.05) m/s 运动
  const Eigen::Matrix3d R_now = (Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(-5.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()))
                                    .toRotationMatrix();
  const Eigen::Vector3d v_body_true(0.4, 0.1, 0.05);
  odom_orientation_ = Eigen::Quaterniond(R_base_lidar.transpose() * R_now * R_base_lidar);
  const Eigen::Vector3d v_c = R_base_lidar.transpose() * (R_now * v_body_true);

  publishOdom(1000.5);
  publishVelocity(1000.5, v_c);
  ASSERT_TRUE(spinUntil([this] { return vision_odom_count_ > 0; }, 5.0));
  ASSERT_TRUE(spinUntil([this] { return pose_cov_count_ > 0; }, 5.0));

  EXPECT_EQ(last_vision_odom_.header.stamp, ros::Time().fromSec(1000.5));
  EXPECT_EQ(last_vision_odom_.header.frame_id, "px4_odom");
  EXPECT_EQ(last_vision_odom_.child_frame_id, "drone_base");
  // 位姿（含协方差）与本地 pose_cov 输出完全一致
  EXPECT_EQ(last_vision_odom_.pose.pose.position.x, last_pose_cov_.pose.pose.position.x);
  EXPECT_EQ(last_vision_odom_.pose.pose.orientation.w, last_pose_cov_.pose.pose.orientation.w);
  for (std::size_t i = 0; i < 36; ++i)
  {
    EXPECT_EQ(last_vision_odom_.pose.covariance[i], last_pose_cov_.pose.covariance[i]) << i;
  }
  const Eigen::Vector3d v_b(last_vision_odom_.twist.twist.linear.x,
                            last_vision_odom_.twist.twist.linear.y,
                            last_vision_odom_.twist.twist.linear.z);
  EXPECT_LT((v_b - v_body_true).norm(), 1e-9) << v_b.transpose();
  EXPECT_TRUE(std::isnan(last_vision_odom_.twist.twist.angular.x));
  EXPECT_DOUBLE_EQ(last_vision_odom_.twist.covariance[0], 0.04);
  EXPECT_DOUBLE_EQ(last_vision_odom_.twist.covariance[7], 0.04);
  EXPECT_DOUBLE_EQ(last_vision_odom_.twist.covariance[14], 0.04);
  EXPECT_EQ(bridge_->visionOdomCount(), 1u);
}

/// odom 方式：速度先到也能配上；配不上速度的位姿不送（计数），时间戳不同的速度不会被套用。
TEST_F(BridgeMessagesTest, OdomModeSendsOnlyMatchedPairs)
{
  startOdomModeBridge();
  publishGravity(tiltedGravity(), 1100.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));

  // 速度先于位姿到达
  publishVelocity(1100.5, Eigen::Vector3d(0.1, 0.0, 0.0));
  spinUntil([] { return false; }, 0.2);
  publishOdom(1100.5);
  ASSERT_TRUE(spinUntil([this] { return vision_odom_count_ == 1; }, 5.0));

  // 1101.0 的位姿没有速度：更晚的速度到了才确定它配不上，记一次缺失，且 1101.0 不会被补发
  publishOdom(1101.0);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ == 2; }, 5.0));
  publishVelocity(1100.9, Eigen::Vector3d(0.1, 0.0, 0.0));  // 没有对应位姿，直接丢掉
  spinUntil([] { return false; }, 0.2);
  EXPECT_EQ(vision_odom_count_, 1);
  EXPECT_EQ(bridge_->missingVelocityCount(), 0u);
  publishOdom(1101.5);
  ASSERT_TRUE(spinUntil([this] { return odom_count_ == 3; }, 5.0));
  publishVelocity(1101.5, Eigen::Vector3d(0.1, 0.0, 0.0));
  ASSERT_TRUE(spinUntil([this] { return vision_odom_count_ == 2; }, 5.0));
  EXPECT_EQ(last_vision_odom_.header.stamp, ros::Time().fromSec(1101.5));
  EXPECT_EQ(bridge_->missingVelocityCount(), 1u);

  // 坐标系不对或非有限的速度被拒绝
  publishOdom(1102.0);
  publishVelocity(1102.0, Eigen::Vector3d(0.1, 0.0, 0.0), "drone_odom");
  publishVelocity(1102.0,
                  Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
  spinUntil([] { return false; }, 0.3);
  EXPECT_EQ(vision_odom_count_, 2);
}

/// odom 方式：两个话题不交替到达（成批、速度一直落后一帧、速度先成批）也全部配上，
/// 发出的时间戳严格递增。
TEST_F(BridgeMessagesTest, OdomModePairsBurstsInAnyOrder)
{
  startOdomModeBridge();
  publishGravity(tiltedGravity(), 1200.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));
  const Eigen::Vector3d v(0.1, 0.0, 0.0);

  // 先连到三条位姿，再连到三条速度
  for (const double s : {1200.1, 1200.2, 1200.3}) feedOdom(s);
  EXPECT_EQ(bridge_->visionOdomCount(), 0u);
  for (const double s : {1200.1, 1200.2, 1200.3}) feedVelocity(s, v);
  EXPECT_EQ(bridge_->visionOdomCount(), 3u);

  // 速度一直落后一帧
  feedOdom(1200.4);
  feedOdom(1200.5);
  feedVelocity(1200.4, v);
  feedOdom(1200.6);
  feedVelocity(1200.5, v);
  feedVelocity(1200.6, v);
  EXPECT_EQ(bridge_->visionOdomCount(), 6u);

  // 速度先连到三条
  for (const double s : {1200.7, 1200.8, 1200.9}) feedVelocity(s, v);
  for (const double s : {1200.7, 1200.8, 1200.9}) feedOdom(s);
  EXPECT_EQ(bridge_->visionOdomCount(), 9u);
  EXPECT_EQ(bridge_->missingVelocityCount(), 0u);

  ASSERT_TRUE(spinUntil([this] { return vision_odom_count_ == 9; }, 5.0));
  for (std::size_t i = 1; i < vision_odom_stamps_.size(); ++i)
  {
    EXPECT_LT(vision_odom_stamps_[i - 1], vision_odom_stamps_[i]) << i;
  }
}

/// odom 方式：丢了一条速度只少发那一帧；时间戳不递增的速度被拒绝；一直没有速度时
/// 队列有界，多出的位姿记缺失，速度恢复后从新的帧接着配。
TEST_F(BridgeMessagesTest, OdomModeDropsOnlyUnpairableData)
{
  startOdomModeBridge();
  publishGravity(tiltedGravity(), 1300.0);
  ASSERT_TRUE(spinUntil([this] { return bridge_->hasAlignment(); }, 5.0));
  const Eigen::Vector3d v(0.1, 0.0, 0.0);

  // 1300.2 的速度丢了
  for (const double s : {1300.1, 1300.2, 1300.3}) feedOdom(s);
  feedVelocity(1300.1, v);
  feedVelocity(1300.3, v);
  EXPECT_EQ(bridge_->visionOdomCount(), 2u);
  EXPECT_EQ(bridge_->missingVelocityCount(), 1u);

  // 1300.4 的速度晚于 1300.5 到达，按乱序拒绝；1300.4 的位姿因此配不上
  feedVelocity(1300.5, v);
  feedVelocity(1300.4, v);
  feedOdom(1300.4);
  EXPECT_EQ(bridge_->missingVelocityCount(), 2u);
  feedOdom(1300.5);
  EXPECT_EQ(bridge_->visionOdomCount(), 3u);

  // 一直没有速度：只留最新的 kMaxPendingPairs 条位姿
  const std::size_t max_pending = fgb::GravityBridge::kMaxPendingPairs;
  const std::size_t n = max_pending + 5;
  for (std::size_t i = 0; i < n; ++i) feedOdom(1301.0 + 0.1 * i);
  EXPECT_EQ(bridge_->visionOdomCount(), 3u);
  EXPECT_EQ(bridge_->missingVelocityCount(), 2u + 5u);

  // 速度恢复：更早的待配对位姿全部记缺失，最新一帧照常发出
  feedVelocity(1301.0 + 0.1 * (n - 1), v);
  EXPECT_EQ(bridge_->visionOdomCount(), 4u);
  EXPECT_EQ(bridge_->missingVelocityCount(), 2u + 5u + (max_pending - 1));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_bridge_messages");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
