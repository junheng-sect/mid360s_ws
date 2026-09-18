#include "fastlio_gravity_bridge/gravity_bridge.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/TransformStamped.h>

namespace fastlio_gravity_bridge
{

GravityBridge::GravityBridge(ros::NodeHandle &nh, const Config &config) : config_(config)
{
  sub_gravity_ = nh.subscribe(config_.gravity_in, 10, &GravityBridge::gravityCallback, this);
  sub_odom_ = nh.subscribe(config_.odom_in, 200, &GravityBridge::odomCallback, this);

  pub_odom_ = nh.advertise<nav_msgs::Odometry>(config_.odom_out, 100);
  pub_pose_cov_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      config_.send_to_mavros ? config_.mavros_pose_cov_topic : config_.pose_cov_out, 100);
  if (config_.send_to_mavros)
  {
    // 目的地改为 MAVROS 时保留一路本地回显，便于板端 / RViz 侧监视（launch 注释有说明）。
    pub_pose_cov_echo_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        config_.pose_cov_out, 100);
  }

  ROS_INFO("[gravity_bridge] in: %s + %s -> out: %s, pose_cov: %s%s",
           config_.odom_in.c_str(), config_.gravity_in.c_str(), config_.odom_out.c_str(),
           config_.send_to_mavros ? config_.mavros_pose_cov_topic.c_str()
                                  : config_.pose_cov_out.c_str(),
           config_.send_to_mavros ? " (+local echo)" : "");
  ROS_INFO("[gravity_bridge] frames: %s -> %s -> %s, static_tf: %s",
           config_.world_frame.c_str(), config_.input_frame.c_str(), config_.child_frame.c_str(),
           config_.publish_static_tf ? "on" : "off");
}

GravityBridge::Config GravityBridge::loadConfig(ros::NodeHandle &pnh)
{
  Config config;
  pnh.param<std::string>("odom_in", config.odom_in, config.odom_in);
  pnh.param<std::string>("gravity_in", config.gravity_in, config.gravity_in);
  pnh.param<std::string>("odom_out", config.odom_out, config.odom_out);
  pnh.param<std::string>("pose_cov_out", config.pose_cov_out, config.pose_cov_out);
  pnh.param<std::string>("world_frame", config.world_frame, config.world_frame);
  pnh.param<std::string>("input_frame", config.input_frame, config.input_frame);
  pnh.param<std::string>("child_frame", config.child_frame, config.child_frame);
  pnh.param<std::string>("mavros_pose_cov_topic", config.mavros_pose_cov_topic,
                         config.mavros_pose_cov_topic);
  pnh.param<bool>("publish_static_tf", config.publish_static_tf, config.publish_static_tf);
  pnh.param<bool>("send_to_mavros", config.send_to_mavros, config.send_to_mavros);
  return config;
}

bool GravityBridge::isSameGravityMessage(const geometry_msgs::Vector3Stamped &msg) const
{
  if (!has_latched_gravity_) return false;
  return msg.header.stamp == latched_gravity_.header.stamp &&
         msg.header.frame_id == latched_gravity_.header.frame_id &&
         msg.vector.x == latched_gravity_.vector.x &&
         msg.vector.y == latched_gravity_.vector.y &&
         msg.vector.z == latched_gravity_.vector.z;
}

void GravityBridge::enterFatalState(const std::string &reason)
{
  if (state_ == State::kFatal) return;
  state_ = State::kFatal;
  fatal_reason_ = reason;
  ROS_ERROR("[gravity_bridge] %s；已停止输出，请随 FAST-LIO 一起重启本节点。", reason.c_str());
}

void GravityBridge::publishStaticTransform()
{
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = latched_gravity_.header.stamp;
  tf_msg.header.frame_id = config_.world_frame;
  tf_msg.child_frame_id = config_.input_frame;
  tf_msg.transform.translation.x = 0.0;
  tf_msg.transform.translation.y = 0.0;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation.w = alignment_.q.w();
  tf_msg.transform.rotation.x = alignment_.q.x();
  tf_msg.transform.rotation.y = alignment_.q.y();
  tf_msg.transform.rotation.z = alignment_.q.z();
  static_tf_broadcaster_.sendTransform(tf_msg);

  const Eigen::AngleAxisd minimal_rotation(alignment_.q);
  // 对齐后 grav 应恰好落在 world 的 -Z 上；残差只用于日志自检。
  const Eigen::Vector3d gravity(latched_gravity_.vector.x, latched_gravity_.vector.y,
                                latched_gravity_.vector.z);
  if (gravity.norm() < kGravityNormEps) return;
  const Eigen::Vector3d residual =
      alignment_.R * gravity.normalized() - Eigen::Vector3d(0.0, 0.0, -1.0);
  ROS_INFO("[gravity_bridge] locked world->%s: q=(%.12f, %.12f, %.12f, %.12f), "
           "minimal rotation %.6f deg about (%.6f, %.6f, %.6f), aligned gravity residual=%.3e",
           config_.input_frame.c_str(), alignment_.q.x(), alignment_.q.y(), alignment_.q.z(),
           alignment_.q.w(), minimal_rotation.angle() * 180.0 / M_PI, minimal_rotation.axis().x(),
           minimal_rotation.axis().y(), minimal_rotation.axis().z(), residual.norm());
}

void GravityBridge::gravityCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
  if (state_ == State::kFatal) return;

  if (has_latched_gravity_)
  {
    // latched 主题只会投递一次；重复投递同一条消息按重复处理，否则视为另一轮初始化。
    if (isSameGravityMessage(*msg)) return;
    enterFatalState("运行中出现另一轮初始化（收到与本次不同的初始化重力）");
    return;
  }

  if (msg->header.frame_id != config_.input_frame)
  {
    ROS_ERROR_THROTTLE(5.0,
                       "[gravity_bridge] 拒绝使用：初始化重力坐标系为 '%s'，期望 '%s'，继续等待。",
                       msg->header.frame_id.c_str(), config_.input_frame.c_str());
    return;
  }

  const Eigen::Vector3d gravity(msg->vector.x, msg->vector.y, msg->vector.z);
  AlignmentTransform candidate;
  if (!computeAlignmentTransform(gravity, candidate))
  {
    ROS_ERROR_THROTTLE(5.0,
                       "[gravity_bridge] 初始化重力无效（非有限值或模长过小: %.6f %.6f %.6f），"
                       "继续等待。",
                       msg->vector.x, msg->vector.y, msg->vector.z);
    return;
  }

  alignment_ = candidate;
  latched_gravity_ = *msg;
  has_latched_gravity_ = true;
  state_ = State::kAligned;

  ROS_INFO("[gravity_bridge] gravity_init (frame_id=%s, t=%.6f): %.6f %.6f %.6f |g|=%.6f",
           msg->header.frame_id.c_str(), msg->header.stamp.toSec(), msg->vector.x,
           msg->vector.y, msg->vector.z, gravity.norm());

  if (config_.publish_static_tf) publishStaticTransform();
}

void GravityBridge::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if (state_ == State::kFatal) return;

  if (state_ == State::kWaitingGravity)
  {
    ROS_INFO_THROTTLE(5.0,
                      "[gravity_bridge] 尚未收到有效的 %s，先不输出未对齐消息。",
                      config_.gravity_in.c_str());
    return;
  }

  if (msg->header.frame_id != config_.input_frame)
  {
    ++rejected_count_;
    ROS_ERROR_THROTTLE(5.0, "[gravity_bridge] 拒绝发布：输入坐标系为 '%s'，期望 '%s'。",
                       msg->header.frame_id.c_str(), config_.input_frame.c_str());
    return;
  }
  if (msg->child_frame_id != config_.child_frame)
  {
    ++rejected_count_;
    ROS_ERROR_THROTTLE(5.0, "[gravity_bridge] 拒绝发布：child_frame_id 为 '%s'，期望 '%s'。",
                       msg->child_frame_id.c_str(), config_.child_frame.c_str());
    return;
  }

  // 早于本次初始化的帧不属于本轮世界系：可能是回放/排队晚到的历史帧，
  // 用本轮的 q_a 处理它们没有意义，直接丢弃（不算时间回退）。
  if (msg->header.stamp < latched_gravity_.header.stamp)
  {
    ++rejected_count_;
    ROS_WARN_THROTTLE(5.0,
                      "[gravity_bridge] 丢弃早于本次初始化的输入（%.9f < %.9f）。",
                      msg->header.stamp.toSec(), latched_gravity_.header.stamp.toSec());
    return;
  }

  if (has_last_published_stamp_)
  {
    if (msg->header.stamp == last_published_stamp_)
    {
      ++rejected_count_;
      ROS_WARN_THROTTLE(5.0, "[gravity_bridge] 丢弃重复时间戳 %.9f 的输入。",
                        msg->header.stamp.toSec());
      return;
    }
    if (msg->header.stamp < last_published_stamp_)
    {
      enterFatalState("输入时间戳回退（" + std::to_string(msg->header.stamp.toSec()) + " < " +
                      std::to_string(last_published_stamp_.toSec()) + "）");
      return;
    }
  }

  if (!poseIsValid(msg->pose.pose))
  {
    ++rejected_count_;
    ROS_ERROR_THROTTLE(5.0, "[gravity_bridge] 拒绝发布：输入位姿含非有限值或无效四元数。");
    return;
  }
  if (!poseCovarianceIsValid(msg->pose.covariance.elems))
  {
    ++rejected_count_;
    ROS_ERROR_THROTTLE(5.0,
                       "[gravity_bridge] 拒绝发布：输入 pose 协方差含非有限值或负方差。");
    return;
  }

  geometry_msgs::PoseWithCovariance transformed;
  if (!transformPoseWithCovariance(alignment_, msg->pose, transformed))
  {
    ++rejected_count_;
    ROS_ERROR_THROTTLE(5.0, "[gravity_bridge] 拒绝发布：位姿与协方差变换失败。");
    return;
  }

  // 只改变父坐标系：twist 及其协方差原样保留（当前 FAST-LIO2 未提供有效速度）。
  nav_msgs::Odometry odom_out = *msg;
  odom_out.header.frame_id = config_.world_frame;
  odom_out.child_frame_id = config_.child_frame;
  odom_out.pose = transformed;
  pub_odom_.publish(odom_out);

  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header = odom_out.header;
  pose_msg.pose = transformed;  // 与变换后 odom 的 pose 完全一致
  pub_pose_cov_.publish(pose_msg);
  if (pub_pose_cov_echo_)
  {
    pub_pose_cov_echo_.publish(pose_msg);
  }

  last_published_stamp_ = msg->header.stamp;
  has_last_published_stamp_ = true;
  ++published_count_;
}

}  // namespace fastlio_gravity_bridge
