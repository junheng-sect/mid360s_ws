#ifndef FASTLIO_GRAVITY_BRIDGE_GRAVITY_BRIDGE_H
#define FASTLIO_GRAVITY_BRIDGE_GRAVITY_BRIDGE_H

#include <cstddef>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "fastlio_gravity_bridge/gravity_alignment.h"

namespace fastlio_gravity_bridge
{

/// 订阅 FAST-LIO2 在 camera_init 下的 /Odometry，用本次运行的初始化重力确定固定对齐
/// 旋转，输出 world 下的里程计与带协方差位姿，并发布 world -> camera_init 静态 TF。
///
/// 每条消息的处理规则：
///  - 尚未锁定对齐旋转：等待，不输出任何未对齐消息；
///  - 早于本次初始化时间的输入：丢弃（不属于本轮世界系的历史帧）；
///  - 非有限值、无效四元数、错误输入坐标系、非法 pose 协方差（非有限、负方差、
///    不对称、非半正定）：拒绝发布并限频报错；
///  - 重复时间戳：丢弃该条消息；
///  - 时间回退或运行中出现另一轮初始化：停止输出（进入 kFatal），需随 FAST-LIO 一起重启。
class GravityBridge
{
 public:
  struct Config
  {
    std::string odom_in = "/Odometry";
    std::string gravity_in = "/fastlio/gravity_init";
    std::string odom_out = "/fastlio/odom_world";
    std::string pose_cov_out = "/fastlio/vision_pose_cov";
    std::string world_frame = "world";
    std::string input_frame = "camera_init";
    std::string child_frame = "body";
    std::string mavros_pose_cov_topic = "/mavros/vision_pose/pose_cov";
    bool publish_static_tf = true;
    bool send_to_mavros = false;
  };

  enum class State
  {
    kWaitingGravity,  ///< 还没收到有效的初始化重力，等待中
    kAligned,         ///< 已锁定本次运行的对齐旋转，正常输出
    kFatal            ///< 需要随 FAST-LIO 一起重启（时间回退 / 出现另一轮初始化）
  };

  GravityBridge(ros::NodeHandle &nh, const Config &config);

  /// 从参数服务器读取配置（参数名与 launch 文件一致）。
  static Config loadConfig(ros::NodeHandle &pnh);

  State state() const { return state_; }
  bool hasAlignment() const { return alignment_.valid; }
  const AlignmentTransform &alignment() const { return alignment_; }
  std::size_t publishedCount() const { return published_count_; }
  const std::string &fatalReason() const { return fatal_reason_; }

  void gravityCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

 private:
  void publishStaticTransform();
  void enterFatalState(const std::string &reason);
  bool isSameGravityMessage(const geometry_msgs::Vector3Stamped &msg) const;

  Config config_;
  AlignmentTransform alignment_;
  State state_ = State::kWaitingGravity;
  std::string fatal_reason_;

  /// 已锁定的初始化重力消息，用于识别重复投递（latched 重发 / 另一轮初始化），
  /// 同时作为本轮输入时间戳的下界（早于它的 odom 一律丢弃）。
  geometry_msgs::Vector3Stamped latched_gravity_;
  bool has_latched_gravity_ = false;

  /// 上一条成功输出的测量时间戳，用于重复时间戳与时间回退判断。
  ros::Time last_published_stamp_;
  bool has_last_published_stamp_ = false;

  std::size_t published_count_ = 0;
  std::size_t rejected_count_ = 0;

  ros::Subscriber sub_gravity_;
  ros::Subscriber sub_odom_;
  ros::Publisher pub_odom_;
  ros::Publisher pub_pose_cov_;
  /// send_to_mavros 时的本地回显发布者（未启用时为空）。
  ros::Publisher pub_pose_cov_echo_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
};

}  // namespace fastlio_gravity_bridge

#endif  // FASTLIO_GRAVITY_BRIDGE_GRAVITY_BRIDGE_H
