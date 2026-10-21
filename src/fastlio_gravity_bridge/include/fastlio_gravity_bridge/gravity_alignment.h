#ifndef FASTLIO_GRAVITY_BRIDGE_GRAVITY_ALIGNMENT_H
#define FASTLIO_GRAVITY_BRIDGE_GRAVITY_ALIGNMENT_H

// 重力对齐与位姿协方差变换的纯数学实现。
// 与 ROS 节点运行时解耦，便于单元测试与文档对照。

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>

namespace fastlio_gravity_bridge
{

/// 四元数模长下限：分量必须有限且模长大于该值，否则视为无效姿态。
constexpr double kQuatNormEps = 1e-6;
/// 重力向量模长下限：正常约 9.81 m/s^2，明显偏小说明输入无效。
constexpr double kGravityNormEps = 1e-6;
/// 输入协方差对角线下限：方差不能为负，仅留数值舍入余量。
constexpr double kCovDiagEps = -1e-12;
/// 协方差对称性与半正定性的相对容差（相对矩阵元素最大绝对值）。
/// 合法协方差的舍入级不对称/负特征值在 1e-15 量级，这里放宽到 1e-9 仍能拦住结构性错误。
constexpr double kCovRelTol = 1e-9;

/// 本次运行的固定重力对齐旋转：把 camera_init 摆到 Z 轴向上的 drone_odom。
/// drone_odom 是局部世界系，原点与 camera_init 相同；只做最小旋转，不调整 yaw，
/// 因此它不代表已经对齐地理东、北。
struct AlignmentTransform
{
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  bool valid = false;
};

inline bool isFiniteVector3(double x, double y, double z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

inline bool isFiniteQuaternion(const Eigen::Quaterniond &q)
{
  return std::isfinite(q.w()) && std::isfinite(q.x()) &&
         std::isfinite(q.y()) && std::isfinite(q.z());
}

/// 姿态有效性：分量有限、模长有限且模长不小于 kQuatNormEps。
inline bool quaternionIsValid(const Eigen::Quaterniond &q)
{
  if (!isFiniteQuaternion(q)) return false;
  const double norm = q.norm();
  return std::isfinite(norm) && norm >= kQuatNormEps;
}

/// 位置有效性：三个分量均为有限值。
inline bool positionIsValid(const geometry_msgs::Point &p)
{
  return isFiniteVector3(p.x, p.y, p.z);
}

inline bool poseIsValid(const geometry_msgs::Pose &pose)
{
  return positionIsValid(pose.position) &&
         quaternionIsValid(Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
                                              pose.orientation.y, pose.orientation.z));
}

/// 按行优先顺序读取消息里的 6x6 位姿协方差。
inline Eigen::Matrix<double, 6, 6> covarianceFromMsg(const double *covariance)
{
  Eigen::Matrix<double, 6, 6> cov;
  for (int i = 0; i < 6; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      cov(i, j) = covariance[i * 6 + j];
    }
  }
  return cov;
}

/// 按行优先顺序写入消息里的 6x6 位姿协方差。
inline void covarianceToMsg(const Eigen::Matrix<double, 6, 6> &cov, double *covariance)
{
  for (int i = 0; i < 6; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      covariance[i * 6 + j] = cov(i, j);
    }
  }
}

/// 仅处理数值舍入造成的不对称：Σ <- (Σ + Σᵀ) / 2，不缩放协方差。
/// 用显式临时量避免 Eigen 别名（直接写 Sigma + Sigma.transpose() 会边读边写）。
inline Eigen::Matrix<double, 6, 6> symmetrizeCovariance(const Eigen::Matrix<double, 6, 6> &cov)
{
  const Eigen::Matrix<double, 6, 6> transposed = cov.transpose();
  return 0.5 * (cov + transposed);
}

/// 协方差合法性判据（都是矩阵本身必须满足的性质）：
///  - 36 个元素均为有限值；
///  - 对角线（方差）不为负（容差 kCovDiagEps）；
///  - 对称：|Σ - Σᵀ| 相对量级不超过 kCovRelTol；
///  - 半正定：最小特征值不小于 -kCovRelTol * |Σ|max。
/// 全零矩阵（本轮 FAST-LIO2 未提供速度时的情况）视为合法。
/// 只做判定，不修改输入、不替换数值。
inline bool covarianceIsValid(const Eigen::Matrix<double, 6, 6> &cov)
{
  for (int i = 0; i < 6; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      if (!std::isfinite(cov(i, j))) return false;
    }
  }

  const double scale = cov.cwiseAbs().maxCoeff();
  if (scale <= 0.0) return true;  // 全零协方差

  const double tol = kCovRelTol * scale;
  for (int i = 0; i < 6; ++i)
  {
    if (cov(i, i) < std::max(kCovDiagEps, -tol)) return false;
  }

  const Eigen::Matrix<double, 6, 6> sym = symmetrizeCovariance(cov);
  if ((cov - sym).cwiseAbs().maxCoeff() > tol) return false;

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(sym);
  if (solver.info() != Eigen::Success) return false;
  return solver.eigenvalues().minCoeff() >= -tol;
}

/// 位姿协方差有效性（消息形式）：转成矩阵后按 covarianceIsValid 判定。
inline bool poseCovarianceIsValid(const double *covariance)
{
  return covarianceIsValid(covarianceFromMsg(covariance));
}

/// 由 camera_init 下的重力向量求本次运行的固定对齐旋转：
/// q_a = FromTwoVectors(g.normalized(), -UnitZ())，把重力方向转到 drone_odom 的 -Z
/// （即 drone_odom 的 +Z 向上）。这是最小旋转，不额外调整 yaw。
/// 输入非有限或模长过小时返回 false，且不修改 out。
inline bool computeAlignmentTransform(const Eigen::Vector3d &gravity,
                                      AlignmentTransform &out)
{
  if (!isFiniteVector3(gravity.x(), gravity.y(), gravity.z())) return false;
  const double norm = gravity.norm();
  if (!std::isfinite(norm) || norm < kGravityNormEps) return false;

  const Eigen::Vector3d g_dir = gravity / norm;
  Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(g_dir, -Eigen::Vector3d::UnitZ());
  if (!quaternionIsValid(q)) return false;
  q.normalize();
  if (!isFiniteQuaternion(q)) return false;

  out.q = q;
  out.R = q.toRotationMatrix();
  out.valid = true;
  return true;
}

/// 机体安装旋转 q_IB（body <- drone_base），作为常量右乘项把输出姿态从雷达 IMU 换成机体：
///   q_out = q_a ⊗ q_CB ⊗ q_IB
/// 方向说明：右乘的 q_IB 作用在雷达 IMU 一侧，把机体系（drone_base）向量转换到雷达 IMU
/// 系，即 q_IB = R_lidar<-drone_base；TF 里发布的 body -> drone_base 就是这个 q_IB
/// （父 body、子 drone_base，四元数表示子系在父系中的姿态）。旧注释写成
/// "drone_base <- body" 是把方向写反了。
///
/// 【旧模式 mounting_source=gravity】由初始化重力推导：
/// body 就是雷达 IMU 坐标系，安装时相对机体有固定倾角（本机约 20 度前倾）。
/// FAST-LIO 初始化时机体保持水平的前提下，对齐旋转 q_a 恰好等于这个安装倾角的
/// 反向，因此取 q_IB = q_a⁻¹：
///   初始化时刻 q_CI = I  ->  q_a ⊗ q_a⁻¹ = I        （姿态归零，水平）
///   之后纯俯仰 φ        ->  R_y(φ)                 （俯仰如实反映）
///   之后纯偏航 ψ        ->  R_z(ψ)                 （偏航如实反映）
/// 前提若被破坏（初始化时机体本身倾斜），输出会把初始倾角一并当作安装角抵消掉，
/// 表现为"读数偏小"，这一点在使用说明与启动日志里都有提示。
/// 已知固定安装角时改用 computeFixedMounting（mounting_source=fixed，默认）。
///
/// 对齐未锁定时返回单位四元数（等价于不做安装补偿）。
inline Eigen::Quaterniond mountingFromAlignment(const AlignmentTransform &alignment)
{
  if (!alignment.valid) return Eigen::Quaterniond::Identity();
  Eigen::Quaterniond q_ib = alignment.q.conjugate();
  q_ib.normalize();
  return q_ib;
}

/// 固定安装旋转（mounting_source=fixed，默认）：从参数指定的安装角直接构造 q_IB，
/// 只由配置决定，**不依赖初始化重力**，因此允许初始化时机体不水平。
///
/// 三个角度（度）描述雷达 IMU 系在机体系（drone_base，FLU）中的姿态，按 ROS 固定轴 RPY
/// （与 tf static_transform_publisher / setRPY 同序）组合：
///   R_base<-lidar = R_z(yaw) · R_y(pitch) · R_x(roll)
///  - pitch 正值 = 前倾（绕 +Y，机头向下）；
///  - roll  正值 = 绕 +X，雷达右侧向下；
///  - yaw   正值 = 绕 +Z，雷达前向朝左偏。
/// 右乘补偿取它的逆：q_IB = (R_base<-lidar)⁻¹；roll = yaw = 0 时就是 R_y(-pitch)。
/// 装配后雷达系看到的"水平前推"因此回到机体系。
/// 任一非有限角度（含换算成弧度后溢出的极端值）返回 false 且不修改 out。
inline bool computeFixedMounting(double mounting_roll_deg, double mounting_pitch_deg,
                                 double mounting_yaw_deg, Eigen::Quaterniond &out)
{
  if (!isFiniteVector3(mounting_roll_deg, mounting_pitch_deg, mounting_yaw_deg)) return false;
  const double roll_rad = mounting_roll_deg * (M_PI / 180.0);
  const double pitch_rad = mounting_pitch_deg * (M_PI / 180.0);
  const double yaw_rad = mounting_yaw_deg * (M_PI / 180.0);
  if (!isFiniteVector3(roll_rad, pitch_rad, yaw_rad)) return false;
  const Eigen::Quaterniond q_base_lidar = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()) *
                                          Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()) *
                                          Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX());
  const Eigen::Quaterniond q = q_base_lidar.conjugate();
  if (!quaternionIsValid(q)) return false;
  out = q.normalized();
  return true;
}

/// 纯俯仰安装（安装横滚、偏航为 0）：q_IB = R_y(-mounting_pitch_deg)。
inline bool computeFixedMounting(double mounting_pitch_deg, Eigen::Quaterniond &out)
{
  return computeFixedMounting(0.0, mounting_pitch_deg, 0.0, out);
}

/// 参考实现：把 IKFoM 前六维状态误差协方差 P6 = [pos, rot] 转换到固定坐标系下
/// 的 ROS 位姿协方差。
///
/// IKFoM 的旋转误差是右扰动（在机体系表达），ROS 要求姿态块按固定轴表达，
/// 因此取 A = diag(I3, R_body_to_frame)，得到 Σ = A P6 Aᵀ：
///   Σ_pp = P_pp              Σ_pθ = P_pθ Rᵀ
///   Σ_θp = R P_θp            Σ_θθ = R P_θθ Rᵀ
/// 交叉项按上述形式完整保留。
///
/// 公式与 fast_lio 包 src/laserMapping.cpp 的 publish_odometry 一致，
/// 本函数用于单元测试与文档对照（含有限差分检查）。
inline Eigen::Matrix<double, 6, 6> ikfomPoseCovarianceToRosFrame(
    const Eigen::Matrix<double, 6, 6> &P6, const Eigen::Matrix3d &R_body_to_frame)
{
  Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
  A.block<3, 3>(3, 3) = R_body_to_frame;
  return symmetrizeCovariance(A * P6 * A.transpose());
}

/// 把 camera_init 下的位姿及协方差变换到 drone_odom：
///   p_W   = R_a p_C
///   q_out = q_a ⊗ q_CB ⊗ q_IB   （左乘对齐、右乘机体安装旋转，最后归一化）
///   Σ_W   = J Σ_C Jᵀ，J = diag(R_a, R_a)
/// 变换后只做舍入层面的对称化。
///
/// q_ib 是 body <- drone_base 的安装旋转：mountingFromAlignment（旧 gravity 模式）或
/// computeFixedMounting（fixed 模式，默认）；不需要安装补偿时传单位四元数。
/// 它是常量右乘，而姿态误差是固定轴左扰动，
/// 因此 δ_out = R_a·δ_C，与加安装旋转之前完全一致 —— 协方差传播不需要改动，
/// 这一点由 PoseCovarianceTransform.MountingRotationKeepsCovariance 固化。
///
/// 输入无效（非有限值、无效四元数、非法协方差）或对齐旋转未锁定时返回 false。
inline bool transformPoseWithCovariance(const AlignmentTransform &transform,
                                       const Eigen::Quaterniond &q_ib,
                                       const geometry_msgs::PoseWithCovariance &in,
                                       geometry_msgs::PoseWithCovariance &out)
{
  if (!transform.valid) return false;
  if (!isFiniteQuaternion(q_ib)) return false;
  if (!positionIsValid(in.pose.position)) return false;

  Eigen::Quaterniond q_cb(in.pose.orientation.w, in.pose.orientation.x,
                          in.pose.orientation.y, in.pose.orientation.z);
  if (!quaternionIsValid(q_cb)) return false;
  q_cb.normalize();

  if (!poseCovarianceIsValid(in.covariance.elems)) return false;

  const Eigen::Vector3d p_c(in.pose.position.x, in.pose.position.y, in.pose.position.z);
  const Eigen::Vector3d p_w = transform.R * p_c;
  // 位置不受安装旋转影响（本轮不做杆臂平移补偿，基点仍是雷达 IMU）。
  const Eigen::Quaterniond q_wb = (transform.q * q_cb * q_ib.normalized()).normalized();
  if (!isFiniteQuaternion(q_wb)) return false;

  out.pose.position.x = p_w.x();
  out.pose.position.y = p_w.y();
  out.pose.position.z = p_w.z();
  out.pose.orientation.w = q_wb.w();
  out.pose.orientation.x = q_wb.x();
  out.pose.orientation.y = q_wb.y();
  out.pose.orientation.z = q_wb.z();

  const Eigen::Matrix<double, 6, 6> cov_c = covarianceFromMsg(in.covariance.elems);
  Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
  J.block<3, 3>(0, 0) = transform.R;
  J.block<3, 3>(3, 3) = transform.R;
  const Eigen::Matrix<double, 6, 6> cov_w = symmetrizeCovariance(J * cov_c * J.transpose());

  // 合法输入在理论上经 J Σ Jᵀ 仍是对称半正定的；这里再校验一次变换结果，
  // 防止今后改动（去掉对称化、额外缩放等）在这里引入非法协方差并对外发布。
  if (!covarianceIsValid(cov_w)) return false;
  covarianceToMsg(cov_w, out.covariance.elems);
  return true;
}

}  // namespace fastlio_gravity_bridge

#endif  // FASTLIO_GRAVITY_BRIDGE_GRAVITY_ALIGNMENT_H
