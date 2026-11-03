#include "fastlio_gravity_bridge/gravity_bridge.h"

#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/TransformStamped.h>

namespace fastlio_gravity_bridge
{

constexpr std::size_t GravityBridge::kMaxPendingPairs;

namespace
{

/// fixed 模式安装角非法时的统一报错文本（构造与参数加载两处共用）。
std::string fixedMountingError(const GravityBridge::Config &config)
{
  return "mounting_roll_deg=" + std::to_string(config.mounting_roll_deg) +
         ", mounting_pitch_deg=" + std::to_string(config.mounting_pitch_deg) +
         ", mounting_yaw_deg=" + std::to_string(config.mounting_yaw_deg) +
         " contains a non-finite angle (fixed mounting)";
}

/// 送飞控方式的配置错误（构造与参数加载两处共用）；合法时返回空串。
std::string evModeError(const GravityBridge::Config &config)
{
  if (!evModeIsKnown(config.ev_mode))
  {
    return "ev_mode='" + config.ev_mode + "' is invalid (expected pose / odom)";
  }
  if (!std::isfinite(config.odom_velocity_std) || config.odom_velocity_std < 0.0)
  {
    return "odom_velocity_std=" + std::to_string(config.odom_velocity_std) +
           " must be finite and >= 0";
  }
  return "";
}

}  // namespace

GravityBridge::GravityBridge(ros::NodeHandle &nh, const Config &config) : config_(config)
{
  // 先把安装旋转定下来：fixed 只取决于参数，与初始化重力无关。配置非法时直接进入
  // kFatal 并返回，连订阅/发布都不建立，避免静默发布错误姿态。
  if (!resolveMounting()) return;
  const std::string ev_error = evModeError(config_);
  if (!ev_error.empty())
  {
    enterFatalState(ev_error);
    return;
  }

  sub_gravity_ = nh.subscribe(config_.gravity_in, 10, &GravityBridge::gravityCallback, this);
  sub_odom_ = nh.subscribe(config_.odom_in, 200, &GravityBridge::odomCallback, this);

  // 位姿只在 pose 方式送 MAVROS；odom 方式位姿随速度一起走 ODOMETRY，绝不两路同时喂飞控。
  const bool odom_mode = config_.ev_mode == "odom";
  const bool pose_to_mavros = config_.send_to_mavros && !odom_mode;
  pub_odom_ = nh.advertise<nav_msgs::Odometry>(config_.odom_out, 100);
  pub_pose_cov_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      pose_to_mavros ? config_.mavros_pose_cov_topic : config_.pose_cov_out, 100);
  if (pose_to_mavros)
  {
    // 目的地改为 MAVROS 时保留一路本地回显，便于板端 / RViz 侧监视（launch 注释有说明）。
    pub_pose_cov_echo_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        config_.pose_cov_out, 100);
  }
  if (odom_mode)
  {
    sub_velocity_ =
        nh.subscribe(config_.velocity_in, 200, &GravityBridge::velocityCallback, this);
    pub_vision_odom_ = nh.advertise<nav_msgs::Odometry>(
        config_.send_to_mavros ? config_.mavros_odom_topic : config_.vision_odom_out, 100);
    if (config_.send_to_mavros)
    {
      pub_vision_odom_echo_ = nh.advertise<nav_msgs::Odometry>(config_.vision_odom_out, 100);
    }
  }

  ROS_INFO("[gravity_bridge] in: %s + %s -> out: %s, pose_cov: %s%s",
           config_.odom_in.c_str(), config_.gravity_in.c_str(), config_.odom_out.c_str(),
           pose_to_mavros ? config_.mavros_pose_cov_topic.c_str() : config_.pose_cov_out.c_str(),
           pose_to_mavros ? " (+local echo)" : "");
  if (odom_mode)
  {
    ROS_INFO("[gravity_bridge] ev_mode=odom: pose + %s -> %s%s (frame %s -> %s, velocity std "
             "%.3f m/s, 0 = EKF2_EVV_NOISE decides)",
             config_.velocity_in.c_str(),
             config_.send_to_mavros ? config_.mavros_odom_topic.c_str()
                                    : config_.vision_odom_out.c_str(),
             config_.send_to_mavros ? " (+local echo)" : "", config_.mavros_odom_frame.c_str(),
             config_.output_child_frame.c_str(), config_.odom_velocity_std);
  }
  else
  {
    ROS_INFO("[gravity_bridge] ev_mode=pose: pose only, no velocity to the flight controller");
  }
  ROS_INFO("[gravity_bridge] frames: %s -> %s -> %s -> %s, static_tf: %s",
           config_.world_frame.c_str(), config_.input_frame.c_str(),
           config_.input_child_frame.c_str(), config_.output_child_frame.c_str(),
           config_.publish_static_tf ? "on" : "off");
}

bool GravityBridge::resolveMounting()
{
  if (!mountingSourceIsKnown(config_.mounting_source))
  {
    enterFatalState("mounting_source='" + config_.mounting_source +
                    "' is invalid (expected fixed / gravity / none)");
    return false;
  }

  if (config_.mounting_source == "fixed")
  {
    Eigen::Quaterniond q_ib;
    if (!computeFixedMounting(config_.mounting_roll_deg, config_.mounting_pitch_deg,
                              config_.mounting_yaw_deg, q_ib))
    {
      enterFatalState(fixedMountingError(config_));
      return false;
    }
    mounting_ = q_ib;
    // fixed 模式只要求初始化时静止（FAST-LIO 需要静态段做重力初始化），不要求水平：
    // 输出保留机体的真实倾角，不再把它当作安装角抵掉。
    ROS_INFO("[gravity_bridge] mounting_source=fixed: LiDAR vs body roll=%.6f pitch=%.6f "
             "yaw=%.6f deg (R_base<-lidar = Rz*Ry*Rx), applying its inverse q_IB=(%.12f, %.12f, "
             "%.12f, %.12f) [xyzw] on the right; independent of init gravity (init must be "
             "static, level is not required).",
             config_.mounting_roll_deg, config_.mounting_pitch_deg, config_.mounting_yaw_deg,
             mounting_.x(), mounting_.y(), mounting_.z(), mounting_.w());
    return true;
  }

  mounting_ = Eigen::Quaterniond::Identity();
  if (config_.mounting_source == "gravity")
  {
    ROS_INFO("[gravity_bridge] mounting_source=gravity: derive q_IB = q_a^-1 from the init "
             "gravity after alignment is locked (assumes the body is level at init).");
  }
  else
  {
    ROS_INFO("[gravity_bridge] mounting_source=none: no mounting compensation, the output "
             "stays the LiDAR IMU attitude.");
  }
  return true;
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
  pnh.param<std::string>("input_child_frame", config.input_child_frame,
                         config.input_child_frame);
  pnh.param<std::string>("output_child_frame", config.output_child_frame,
                         config.output_child_frame);
  pnh.param<std::string>("mounting_source", config.mounting_source, config.mounting_source);
  pnh.param<double>("mounting_roll_deg", config.mounting_roll_deg, config.mounting_roll_deg);
  pnh.param<double>("mounting_pitch_deg", config.mounting_pitch_deg, config.mounting_pitch_deg);
  pnh.param<double>("mounting_yaw_deg", config.mounting_yaw_deg, config.mounting_yaw_deg);
  pnh.param<std::string>("mavros_pose_cov_topic", config.mavros_pose_cov_topic,
                         config.mavros_pose_cov_topic);
  pnh.param<bool>("publish_static_tf", config.publish_static_tf, config.publish_static_tf);
  pnh.param<bool>("send_to_mavros", config.send_to_mavros, config.send_to_mavros);
  pnh.param<std::string>("ev_mode", config.ev_mode, config.ev_mode);
  pnh.param<std::string>("velocity_in", config.velocity_in, config.velocity_in);
  pnh.param<std::string>("vision_odom_out", config.vision_odom_out, config.vision_odom_out);
  pnh.param<std::string>("mavros_odom_topic", config.mavros_odom_topic,
                         config.mavros_odom_topic);
  pnh.param<std::string>("mavros_odom_frame", config.mavros_odom_frame,
                         config.mavros_odom_frame);
  pnh.param<double>("odom_velocity_std", config.odom_velocity_std, config.odom_velocity_std);

  const std::string ev_error = evModeError(config);
  if (!ev_error.empty())
  {
    config.valid = false;
    config.error = ev_error;
    ROS_ERROR("[gravity_bridge] %s; refusing to start.", config.error.c_str());
    return config;
  }

  // 安装旋转的非法配置不再静默回退（旧实现会退回 none 并发出一条告警），
  // 而是标记配置非法；节点入口据此拒绝启动，避免对外发布错误姿态。
  if (!mountingSourceIsKnown(config.mounting_source))
  {
    config.valid = false;
    config.error = "mounting_source='" + config.mounting_source +
                   "' is invalid (expected fixed / gravity / none)";
    ROS_ERROR("[gravity_bridge] %s; refusing to start.", config.error.c_str());
    return config;
  }
  if (config.mounting_source == "fixed")
  {
    Eigen::Quaterniond q_ib;
    if (!computeFixedMounting(config.mounting_roll_deg, config.mounting_pitch_deg,
                              config.mounting_yaw_deg, q_ib))
    {
      config.valid = false;
      config.error = fixedMountingError(config);
      ROS_ERROR("[gravity_bridge] %s; refusing to start.", config.error.c_str());
      return config;
    }
  }
  if (config.output_child_frame == config.input_child_frame)
  {
    ROS_WARN("[gravity_bridge] output_child_frame equals input_child_frame (%s): with mounting "
             "compensation enabled the attitude will not match the frame name.",
             config.output_child_frame.c_str());
  }
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
  ROS_ERROR("[gravity_bridge] %s; stopping output, restart this node together with FAST-LIO.",
            reason.c_str());
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

  // 安装旋转另发一段静态 TF，使 TF 树与 odom 消息自洽：
  //   drone_odom -> camera_init -> body(=雷达 IMU) -> drone_base(=机体)
  // 平移为零：drone_base 的原点与雷达 IMU 重合，杆臂不在本节点补偿。
  // 只要输出 child frame 与输入不同名就发布，旋转就是位姿输出用的同一个 mounting_
  // （fixed / gravity 为安装旋转，none 为单位四元数），保证两者永远一致。
  if (config_.output_child_frame != config_.input_child_frame)
  {
    geometry_msgs::TransformStamped mount_msg;
    mount_msg.header.stamp = latched_gravity_.header.stamp;
    mount_msg.header.frame_id = config_.input_child_frame;
    mount_msg.child_frame_id = config_.output_child_frame;
    mount_msg.transform.translation.x = 0.0;
    mount_msg.transform.translation.y = 0.0;
    mount_msg.transform.translation.z = 0.0;
    mount_msg.transform.rotation.w = mounting_.w();
    mount_msg.transform.rotation.x = mounting_.x();
    mount_msg.transform.rotation.y = mounting_.y();
    mount_msg.transform.rotation.z = mounting_.z();
    static_tf_broadcaster_.sendTransform(mount_msg);
  }

  const Eigen::AngleAxisd minimal_rotation(alignment_.q);
  // 对齐后 grav 应恰好落在 drone_odom 的 -Z 上；残差只用于日志自检。
  const Eigen::Vector3d gravity(latched_gravity_.vector.x, latched_gravity_.vector.y,
                                latched_gravity_.vector.z);
  if (gravity.norm() < kGravityNormEps) return;
  const Eigen::Vector3d residual =
      alignment_.R * gravity.normalized() - Eigen::Vector3d(0.0, 0.0, -1.0);
  ROS_INFO("[gravity_bridge] locked %s->%s: q=(%.12f, %.12f, %.12f, %.12f), "
           "minimal rotation %.6f deg about (%.6f, %.6f, %.6f), aligned gravity residual=%.3e",
           config_.world_frame.c_str(), config_.input_frame.c_str(), alignment_.q.x(),
           alignment_.q.y(), alignment_.q.z(), alignment_.q.w(),
           minimal_rotation.angle() * 180.0 / M_PI, minimal_rotation.axis().x(),
           minimal_rotation.axis().y(), minimal_rotation.axis().z(), residual.norm());
}

void GravityBridge::gravityCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
  if (state_ == State::kFatal) return;

  if (has_latched_gravity_)
  {
    // latched 主题只会投递一次；重复投递同一条消息按重复处理，否则视为另一轮初始化。
    if (isSameGravityMessage(*msg)) return;
    enterFatalState("a second initialization appeared (gravity_init differs from this run's)");
    return;
  }

  if (msg->header.frame_id != config_.input_frame)
  {
    ROS_ERROR_THROTTLE(5.0,
                       "[gravity_bridge] rejecting gravity_init: frame_id is '%s', expected "
                       "'%s'; still waiting.",
                       msg->header.frame_id.c_str(), config_.input_frame.c_str());
    return;
  }

  const Eigen::Vector3d gravity(msg->vector.x, msg->vector.y, msg->vector.z);
  AlignmentTransform candidate;
  if (!computeAlignmentTransform(gravity, candidate))
  {
    ROS_ERROR_THROTTLE(5.0,
                       "[gravity_bridge] invalid gravity_init (non-finite or too small: "
                       "%.6f %.6f %.6f); still waiting.",
                       msg->vector.x, msg->vector.y, msg->vector.z);
    return;
  }

  alignment_ = candidate;
  // fixed 的安装旋转已在构造时按参数确定，与初始化重力无关；none 保持单位四元数。
  if (config_.mounting_source == "gravity")
  {
    mounting_ = mountingFromAlignment(alignment_);
  }
  latched_gravity_ = *msg;
  has_latched_gravity_ = true;
  state_ = State::kAligned;

  ROS_INFO("[gravity_bridge] gravity_init (frame_id=%s, t=%.6f): %.6f %.6f %.6f |g|=%.6f",
           msg->header.frame_id.c_str(), msg->header.stamp.toSec(), msg->vector.x,
           msg->vector.y, msg->vector.z, gravity.norm());

  if (config_.mounting_source == "gravity")
  {
    const Eigen::AngleAxisd mount_angle(mounting_);
    ROS_INFO("[gravity_bridge] mounting %s->%s (legacy gravity mode, q_IB = q_a^-1): cancels "
             "%.6f deg of sensor tilt about (%.6f, %.6f, %.6f) (assumes body level at init)",
             config_.input_child_frame.c_str(), config_.output_child_frame.c_str(),
             mount_angle.angle() * 180.0 / M_PI, mount_angle.axis().x(), mount_angle.axis().y(),
             mount_angle.axis().z());
  }
  else if (config_.mounting_source == "fixed")
  {
    // fixed 模式的安装旋转与初始化重力无关，真正的倾角由输出姿态如实保留。
    const Eigen::AngleAxisd tilt(alignment_.q);
    ROS_INFO("[gravity_bridge] fixed mounting %s->%s kept unchanged (roll=%.6f pitch=%.6f "
             "yaw=%.6f deg); init gravity tilt %.6f deg about (%.6f, %.6f, %.6f) is preserved "
             "in the output",
             config_.input_child_frame.c_str(), config_.output_child_frame.c_str(),
             config_.mounting_roll_deg, config_.mounting_pitch_deg, config_.mounting_yaw_deg,
             tilt.angle() * 180.0 / M_PI, tilt.axis().x(), tilt.axis().y(), tilt.axis().z());
  }

  if (config_.publish_static_tf) publishStaticTransform();
}

void GravityBridge::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if (state_ == State::kFatal) return;

  if (state_ == State::kWaitingGravity)
  {
    ROS_INFO_THROTTLE(5.0, "[gravity_bridge] no valid %s yet; not publishing unaligned data.",
                      config_.gravity_in.c_str());
    return;
  }

  if (msg->header.frame_id != config_.input_frame)
  {
    ++rejected_count_;
    ROS_ERROR_THROTTLE(5.0, "[gravity_bridge] rejecting input: frame_id is '%s', expected '%s'.",
                       msg->header.frame_id.c_str(), config_.input_frame.c_str());
    return;
  }
  if (msg->child_frame_id != config_.input_child_frame)
  {
    ++rejected_count_;
    ROS_ERROR_THROTTLE(5.0,
                       "[gravity_bridge] rejecting input: child_frame_id is '%s', expected '%s'.",
                       msg->child_frame_id.c_str(), config_.input_child_frame.c_str());
    return;
  }

  // 早于本次初始化的帧不属于本轮世界系：可能是回放/排队晚到的历史帧，
  // 用本轮的 q_a 处理它们没有意义，直接丢弃（不算时间回退）。
  if (msg->header.stamp < latched_gravity_.header.stamp)
  {
    ++rejected_count_;
    ROS_WARN_THROTTLE(5.0,
                      "[gravity_bridge] dropping input older than this initialization "
                      "(%.9f < %.9f).",
                      msg->header.stamp.toSec(), latched_gravity_.header.stamp.toSec());
    return;
  }

  if (has_last_published_stamp_)
  {
    if (msg->header.stamp == last_published_stamp_)
    {
      ++rejected_count_;
      ROS_WARN_THROTTLE(5.0, "[gravity_bridge] dropping duplicate timestamp %.9f.",
                        msg->header.stamp.toSec());
      return;
    }
    if (msg->header.stamp < last_published_stamp_)
    {
      enterFatalState("input timestamp went backwards (" +
                      std::to_string(msg->header.stamp.toSec()) + " < " +
                      std::to_string(last_published_stamp_.toSec()) + ")");
      return;
    }
  }

  if (!poseIsValid(msg->pose.pose))
  {
    ++rejected_count_;
    ROS_ERROR_THROTTLE(5.0,
                       "[gravity_bridge] rejecting input: pose has non-finite values or an "
                       "invalid quaternion.");
    return;
  }
  if (!poseCovarianceIsValid(msg->pose.covariance.elems))
  {
    ++rejected_count_;
    ROS_ERROR_THROTTLE(5.0,
                       "[gravity_bridge] rejecting input: pose covariance is non-finite, "
                       "asymmetric or not positive semi-definite.");
    return;
  }

  geometry_msgs::PoseWithCovariance transformed;
  if (!transformPoseWithCovariance(alignment_, mounting_, msg->pose, transformed))
  {
    ++rejected_count_;
    ROS_ERROR_THROTTLE(5.0, "[gravity_bridge] rejecting input: pose/covariance transform failed.");
    return;
  }

  // 只改变父/子坐标系：twist 及其协方差原样保留（当前 FAST-LIO2 未提供有效速度）。
  // 位置基点仍是雷达 IMU，姿态已是机体（安装旋转见 publishStaticTransform）。
  nav_msgs::Odometry odom_out = *msg;
  odom_out.header.frame_id = config_.world_frame;
  odom_out.child_frame_id = config_.output_child_frame;
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

  if (config_.ev_mode == "odom")
  {
    // 进到这里的位姿时间戳严格递增（重复已丢、回退已进 kFatal），队列天然有序。
    pending_poses_.push_back(pose_msg);
    tryPublishVisionOdom();
  }
}

void GravityBridge::velocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
  if (state_ == State::kFatal) return;

  if (msg->header.frame_id != config_.input_frame)
  {
    ++rejected_count_;
    ROS_ERROR_THROTTLE(5.0,
                       "[gravity_bridge] rejecting velocity: frame_id is '%s', expected '%s'.",
                       msg->header.frame_id.c_str(), config_.input_frame.c_str());
    return;
  }
  if (!isFiniteVector3(msg->vector.x, msg->vector.y, msg->vector.z))
  {
    ++rejected_count_;
    ROS_ERROR_THROTTLE(5.0, "[gravity_bridge] rejecting velocity: non-finite values.");
    return;
  }

  // 归并配对依赖速度按时间戳升序；不递增的（FAST-LIO 重启、重复投递）拒绝。
  if (!pending_velocities_.empty() &&
      msg->header.stamp <= pending_velocities_.back().header.stamp)
  {
    ++rejected_count_;
    ROS_ERROR_THROTTLE(5.0,
                       "[gravity_bridge] rejecting velocity: stamp %.9f is not after the previous "
                       "one %.9f.",
                       msg->header.stamp.toSec(),
                       pending_velocities_.back().header.stamp.toSec());
    return;
  }

  pending_velocities_.push_back(*msg);
  tryPublishVisionOdom();
}

void GravityBridge::tryPublishVisionOdom()
{
  // 两个队列各自按时间戳升序（同一话题内按序到达），按时间戳归并：
  //  - 相同：配成一条发出；
  //  - 位姿更早：它的速度本该排在队首速度之前，已不会再来，这条位姿不送飞控；
  //  - 速度更早：它的位姿没有输出（被拒绝或丢了），丢掉这条速度。
  // 发出的消息因此时间戳严格递增，不会在新的之后补发旧的。
  while (!pending_poses_.empty() && !pending_velocities_.empty())
  {
    const ros::Time pose_stamp = pending_poses_.front().header.stamp;
    const ros::Time velocity_stamp = pending_velocities_.front().header.stamp;
    if (pose_stamp == velocity_stamp)
    {
      publishVisionOdom(pending_poses_.front(), pending_velocities_.front());
      pending_poses_.pop_front();
      pending_velocities_.pop_front();
    }
    else if (pose_stamp < velocity_stamp)
    {
      dropPoseWithoutVelocity();
    }
    else
    {
      pending_velocities_.pop_front();
    }
  }
  // 一边一直不来（例如 FAST-LIO 没带 /fastlio/velocity）时队列不能无限长：丢最旧的。
  while (pending_poses_.size() > kMaxPendingPairs) dropPoseWithoutVelocity();
  while (pending_velocities_.size() > kMaxPendingPairs) pending_velocities_.pop_front();
}

void GravityBridge::dropPoseWithoutVelocity()
{
  ++missing_velocity_count_;
  ROS_ERROR_THROTTLE(5.0,
                     "[gravity_bridge] ev_mode=odom: no %s with stamp %.9f, that pose was NOT "
                     "sent to the flight controller (%zu missing so far).",
                     config_.velocity_in.c_str(), pending_poses_.front().header.stamp.toSec(),
                     missing_velocity_count_);
  pending_poses_.pop_front();
}

void GravityBridge::publishVisionOdom(const geometry_msgs::PoseWithCovarianceStamped &pose,
                                      const geometry_msgs::Vector3Stamped &velocity)
{
  Eigen::Vector3d v_b;
  const Eigen::Vector3d v_c(velocity.vector.x, velocity.vector.y, velocity.vector.z);
  if (!transformVelocityToBody(alignment_, pose.pose.pose.orientation, v_c, v_b))
  {
    ++rejected_count_;
    ROS_ERROR_THROTTLE(5.0, "[gravity_bridge] rejecting velocity: transform failed.");
    return;
  }

  // nav_msgs/Odometry 约定：pose 在 header.frame_id 下，twist 在 child_frame_id（机体）下。
  // 角速度 FAST-LIO 没给，填 NaN 表示未知（PX4 的 EKF 不用外部角速度）。
  nav_msgs::Odometry out;
  out.header.stamp = pose.header.stamp;
  out.header.frame_id = config_.mavros_odom_frame;
  out.child_frame_id = config_.output_child_frame;
  out.pose = pose.pose;
  out.twist.twist.linear.x = v_b.x();
  out.twist.twist.linear.y = v_b.y();
  out.twist.twist.linear.z = v_b.z();
  out.twist.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  out.twist.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  out.twist.twist.angular.z = std::numeric_limits<double>::quiet_NaN();
  const double velocity_var = config_.odom_velocity_std * config_.odom_velocity_std;
  out.twist.covariance[0] = velocity_var;
  out.twist.covariance[7] = velocity_var;
  out.twist.covariance[14] = velocity_var;

  pub_vision_odom_.publish(out);
  if (pub_vision_odom_echo_)
  {
    pub_vision_odom_echo_.publish(out);
  }
  ++vision_odom_count_;
}

}  // namespace fastlio_gravity_bridge
