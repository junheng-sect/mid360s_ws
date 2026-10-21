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

/// 安装旋转来源的合法取值。非法取值不再静默回退，而是让启动失败（见 loadConfig）。
inline bool mountingSourceIsKnown(const std::string &source)
{
  return source == "fixed" || source == "gravity" || source == "none";
}

/// 订阅 FAST-LIO2 在 camera_init 下的 /Odometry，用本次运行的初始化重力确定固定对齐
/// 旋转，输出 drone_odom 下的里程计与带协方差位姿，并发布静态 TF：
///   drone_odom -> camera_init（对齐旋转 q_a）与 body -> drone_base（机体安装旋转 q_IB）。
///
/// 输出姿态表示 drone_base（机体）而不是雷达 IMU；位置基点仍是雷达 IMU
/// （本轮不做杆臂补偿，由下游/飞控参数处理）。
///
/// 安装旋转 q_IB（body <- drone_base）由 mounting_source 决定：
///  - fixed（默认）：用 mounting_roll/pitch/yaw_deg 指定的固定安装旋转，不依赖初始化重力，
///    因此初始化时机体**保持静止即可，不要求水平**；
///  - gravity：由初始化重力推导 q_IB = q_a⁻¹，前提是初始化时机体水平；
///  - none：不做安装补偿，输出仍是雷达 IMU 姿态。
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
    std::string world_frame = "drone_odom";
    std::string input_frame = "camera_init";
    /// 输入校验用的 child frame：FAST-LIO2 发布的就是 body（雷达 IMU）。
    std::string input_child_frame = "body";
    /// 输出命名用的 child frame：姿态已换成机体，故与输入不同名。
    std::string output_child_frame = "drone_base";
    /// 机体安装旋转的来源："fixed"（默认）= 用下方的固定安装角，与初始化姿态无关；
    /// "gravity" = 由初始化重力推导 q_IB = q_a⁻¹（前提：初始化时机体水平）；
    /// "none" = 不做安装补偿（输出仍是雷达 IMU 姿态）。
    std::string mounting_source = "fixed";
    /// 雷达 IMU 相对机体的安装姿态（度），仅 mounting_source=fixed 生效：
    /// R_base<-lidar = R_z(yaw)·R_y(pitch)·R_x(roll)，内部取逆作为右乘补偿 q_IB。
    /// pitch 正值 = 前倾；roll 正值 = 雷达右侧向下；yaw 正值 = 雷达前向朝左偏。
    /// roll、yaw 默认 0，即纯俯仰 q_IB = R_y(-mounting_pitch_deg)。
    double mounting_roll_deg = 0.0;
    double mounting_pitch_deg = 20.0;
    double mounting_yaw_deg = 0.0;
    std::string mavros_pose_cov_topic = "/mavros/vision_pose/pose_cov";
    bool publish_static_tf = true;
    bool send_to_mavros = false;
    /// 参数校验结果：false 时 error 给出原因，节点入口据此拒绝启动（不发布任何姿态）。
    bool valid = true;
    std::string error;
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
  /// body <- drone_base 的安装旋转（把机体系向量转到雷达 IMU 系；none 时为单位四元数）。
  const Eigen::Quaterniond &mounting() const { return mounting_; }
  std::size_t publishedCount() const { return published_count_; }
  const std::string &fatalReason() const { return fatal_reason_; }

  void gravityCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

 private:
  void publishStaticTransform();
  void enterFatalState(const std::string &reason);
  /// 按 mounting_source 解析本轮的安装旋转：fixed 在构造时确定，gravity 等初始化重力，
  /// none 保持单位四元数。配置非法时进入 kFatal（不发布任何姿态）。
  bool resolveMounting();
  bool isSameGravityMessage(const geometry_msgs::Vector3Stamped &msg) const;

  Config config_;
  AlignmentTransform alignment_;
  /// body <- drone_base：由 mounting_source 决定；fixed 在构造时确定，gravity 在锁定
  /// 对齐旋转时一并确定，none 始终为单位四元数。
  Eigen::Quaterniond mounting_ = Eigen::Quaterniond::Identity();
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
