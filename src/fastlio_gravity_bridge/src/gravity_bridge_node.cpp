// fastlio_gravity_bridge 节点入口。
// 订阅 FAST-LIO2 的 /Odometry 与初始化重力，输出重力对齐的 odom / pose_cov，
// 并发布 world -> camera_init 静态 TF。参数说明见 launch/gravity_bridge.launch。

#include <ros/ros.h>

#include "fastlio_gravity_bridge/gravity_bridge.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gravity_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  const fastlio_gravity_bridge::GravityBridge::Config config =
      fastlio_gravity_bridge::GravityBridge::loadConfig(pnh);
  if (!config.valid)
  {
    // 非法安装配置（未知 mounting_source、非有限角度）：明确报错并停止启动，
    // 绝不带着兜底值继续对外发布姿态。
    ROS_FATAL("[gravity_bridge] invalid configuration: %s; refusing to start.",
              config.error.c_str());
    return 1;
  }

  fastlio_gravity_bridge::GravityBridge bridge(nh, config);
  ros::spin();
  return 0;
}
