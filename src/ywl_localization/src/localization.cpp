// Adapted from YWL0720/FAST-LOCALIZATION (GPL-2.0); see UPSTREAM.md.
#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/make_shared.hpp>
#include <ros/callback_queue.h>
#include <pcl/common/point_tests.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <openssl/evp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "IMU_Processing.hpp"
#include "ikd-Tree/ikd_Tree.h"
#include "preprocess.h"

namespace {
using Clock = std::chrono::steady_clock;
using Pose = Eigen::Isometry3d;
constexpr double kFresh = 0.5;
constexpr double kLaserVariance = 0.001;  // YWL upstream
constexpr char kMap[] = "drone_map";
constexpr char kLocal[] = "ywl_odom";
constexpr char kImu[] = "ywl_imu";

double elapsed(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

std::string sha256(const std::string &path) {
    std::ifstream input(path, std::ios::binary);
    std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
    if (!input || !ctx || EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) != 1) {
        throw std::runtime_error("cannot open or hash " + path);
    }
    std::array<char, 65536> data;
    while (input.read(data.data(), data.size()) || input.gcount()) {
        if (EVP_DigestUpdate(ctx.get(), data.data(), input.gcount()) != 1)
            throw std::runtime_error("cannot hash " + path);
    }
    std::array<unsigned char, 32> digest;
    unsigned int size;
    if (!input.eof() || input.bad() || EVP_DigestFinal_ex(ctx.get(), digest.data(), &size) != 1 ||
        size != digest.size()) throw std::runtime_error("cannot hash " + path);
    std::ostringstream out;
    for (unsigned char byte : digest) out << std::hex << std::setw(2) << std::setfill('0') << int(byte);
    return out.str();
}

std::array<long long, 7> file_version(const std::string &path) {
    struct stat s {};
    if (stat(path.c_str(), &s) || !S_ISREG(s.st_mode))
        throw std::runtime_error("expected regular file " + path);
    return {{static_cast<long long>(s.st_dev), static_cast<long long>(s.st_ino), s.st_size,
             s.st_mtim.tv_sec, s.st_mtim.tv_nsec, s.st_ctim.tv_sec, s.st_ctim.tv_nsec}};
}

geometry_msgs::Quaternion quaternion(const M3D &rotation) {
    const Eigen::Quaterniond q(rotation);
    geometry_msgs::Quaternion msg;
    msg.x = q.x(); msg.y = q.y(); msg.z = q.z(); msg.w = q.w();
    return msg;
}

void check_finite(const geometry_msgs::Vector3 &v) {
    if (!V3D(v.x, v.y, v.z).allFinite()) throw std::runtime_error("nonfinite IMU input");
}

class Node {
  public:
    Node() : pnh_("~") {
        status_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/ywl/status", 10, true);
    }

    void start() {
        try {
            configure();
        } catch (const std::exception &exc) {
            fail("FAILED", exc.what());
        }
        watch_ = watchdog_nh_.createSteadyTimer(ros::WallDuration(0.05), &Node::watchdog, this);
    }

    void run() {
        watchdog_nh_.setCallbackQueue(&watchdog_queue_);
        start();
        ros::AsyncSpinner watchdog_spinner(1, &watchdog_queue_);
        watchdog_spinner.start();
        ros::WallRate loop(1000);
        while (ros::ok()) {
            ros::spinOnce();
            if (!terminal() && !scans_.empty() && last_imu_stamp_ >= scans_.front().end) {
                try {
                    process_scan();
                } catch (const std::exception &exc) {
                    fail("FAILED", exc.what());
                }
            } else {
                loop.sleep();
            }
        }
    }

  private:
    struct Scan {
        PointCloudXYZI::Ptr cloud;
        double begin, end;
        Clock::time_point received;
    };
    struct Stats {
        int points = 0, matches = 0, map_points = 0, backlog = 0;
        double residual = 0, work_ms = 0, input_ms = 0, queue_ms = 0;
        double pos_update = 0, rot_update = 0;
        uint64_t lidar = 0, processed = 0, published = 0;
    };

    void configure() {
        std::string map_dir;
        if (!pnh_.getParam("map_dir", map_dir) || map_dir.empty())
            throw std::runtime_error("~map_dir is required");
        pcd_ = map_dir + "/map_world.pcd";
        meta_ = map_dir + "/meta.json";
        pcd_version_ = file_version(pcd_);
        meta_version_ = file_version(meta_);
        pcd_sha_ = sha256(pcd_);
        meta_sha_ = sha256(meta_);
        boost::property_tree::ptree metadata;
        boost::property_tree::read_json(meta_, metadata);
        PointCloudXYZI prior;
        if (metadata.get<uint64_t>("points_world") < NUM_MATCH_POINTS ||
            pcl::io::loadPCDFile(pcd_, prior) == -1 || prior.size() < NUM_MATCH_POINTS ||
            prior.size() != metadata.get<uint64_t>("points_world"))
            throw std::runtime_error("invalid or empty prior map");
        for (const auto &point : prior) {
            if (!pcl::isFinite(point)) throw std::runtime_error("nonfinite prior point");
        }
        fixed_tree_ = std::make_unique<KD_TREE<PointType>>();
        fixed_tree_->Build(PointVector(prior.begin(), prior.end()));
        if (file_version(pcd_) != pcd_version_ || file_version(meta_) != meta_version_)
            throw std::runtime_error("map changed while loading");
        std::vector<double> translation, rotation;
        if (!pnh_.getParam("extrinsic_T", translation) || translation.size() != 3 ||
            !pnh_.getParam("extrinsic_R", rotation) || rotation.size() != 9)
            throw std::runtime_error("missing LiDAR to IMU calibration");
        extrinsic_t_ = Eigen::Map<V3D>(translation.data());
        extrinsic_r_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotation.data());
        if (!extrinsic_t_.allFinite() || !extrinsic_r_.allFinite() ||
            !(extrinsic_r_.transpose() * extrinsic_r_).isApprox(M3D::Identity(), 1e-6) ||
            std::abs(extrinsic_r_.determinant() - 1) > 1e-6)
            throw std::runtime_error("invalid LiDAR to IMU calibration");
        pnh_.param("time_offset_lidar_to_imu", time_offset_, 0.0);
        pnh_.param("publish_cloud", publish_cloud_, false);
        double blind, surf_leaf;
        int decimation, iterations;
        pnh_.param("blind", blind, 0.5);
        pnh_.param("scan_lines", preprocess_.N_SCANS, 4);
        pnh_.param("point_filter_num", decimation, 3);
        pnh_.param("max_iterations", iterations, 3);
        pnh_.param("filter_size_surf", surf_leaf, 0.5);
        pnh_.param("filter_size_bootstrap", map_leaf_, 0.5);
        if (!std::isfinite(time_offset_) || !std::isfinite(blind) || blind <= 0 ||
            !std::isfinite(surf_leaf) || surf_leaf <= 0 || !std::isfinite(map_leaf_) || map_leaf_ <= 0 ||
            decimation < 1 || preprocess_.N_SCANS < 1 || preprocess_.N_SCANS > 128 ||
            iterations < 1 || iterations > 10)
            throw std::runtime_error("invalid MID360S parameters");
        preprocess_.set(false, AVIA, blind, decimation);
        voxel_.setLeafSize(surf_leaf, surf_leaf, surf_leaf);
        imu_.set_extrinsic(extrinsic_t_, extrinsic_r_);
        imu_.set_acc_cov(noise("acc_cov", 0.1));
        imu_.set_gyr_cov(noise("gyr_cov", 0.1));
        imu_.set_acc_bias_cov(noise("b_acc_cov", 0.0001));
        imu_.set_gyr_bias_cov(noise("b_gyr_cov", 0.0001));

        // ROS parameter updates are checked by the independent watchdog.
        for (const auto *key : {"map_dir", "extrinsic_T", "extrinsic_R", "time_offset_lidar_to_imu"}) {
            XmlRpc::XmlRpcValue value;
            pnh_.getParamCached(key, value);
            config_snapshot_.emplace_back(key, value);
        }
        double eps[23];
        std::fill(std::begin(eps), std::end(eps), 0.001);
        active_node_ = this;  // IKFoM takes a plain function pointer. One estimator per process.
        filter_.init_dyn_share(get_f, df_dx, df_dw, &Node::observe, iterations, eps);
        bootstrap_ = nh_.advertise<nav_msgs::Odometry>("/ywl/bootstrap_odom", 10);
        odom_ = nh_.advertise<nav_msgs::Odometry>("/ywl/odom", 10);
        velocity_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/ywl/velocity", 10);
        if (publish_cloud_) registered_ = nh_.advertise<sensor_msgs::PointCloud2>("/ywl/cloud_registered", 2);
        lidar_sub_ = nh_.subscribe("/livox/lidar", 8, &Node::lidar, this, ros::TransportHints().tcpNoDelay());
        imu_sub_ = nh_.subscribe("/livox/imu", 400, &Node::imu, this, ros::TransportHints().tcpNoDelay());
        init_sub_ = nh_.subscribe("/ywl/initial_transform", 1, &Node::initial_transform, this);
        configured_ = true;
    }

    V3D noise(const std::string &key, double fallback) {
        double value;
        pnh_.param(key, value, fallback);
        if (!std::isfinite(value) || value <= 0) throw std::runtime_error("invalid " + key);
        return V3D::Constant(value);
    }

    bool terminal() {
        const std::lock_guard<std::mutex> lock(health_);
        return state_ == "STALE" || state_ == "FAILED";
    }

    void fail(const std::string &state, const std::string &reason) {
        const std::lock_guard<std::mutex> lock(health_);
        fail_locked(state, reason);
    }

    void fail_locked(const std::string &state, const std::string &reason) {
        if (state_ == "STALE" || state_ == "FAILED") return;
        state_ = state;
        reason_ = reason;
        ROS_ERROR_STREAM("YWL " << state_ << ": " << reason);
        diagnostic_locked();
    }

    void lidar(const livox_ros_driver2::CustomMsg::ConstPtr &msg) {
        const auto received = Clock::now();
        if (terminal()) return;
        const double begin = msg->header.stamp.toSec();
        if (begin <= last_lidar_stamp_ || begin <= 0) {
            fail("FAILED", "LiDAR timestamp repeated or reversed");
            return;
        }
        if (msg->point_num != msg->points.size() || msg->point_num > 200000 ||
            std::any_of(msg->points.begin(), msg->points.end(), [](const auto &p) {
                return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
                       p.offset_time > 500000000;
            })) {
            fail("FAILED", "invalid LiDAR scan");
            return;
        }
        if (scans_.size() >= 8) {
            fail("STALE", "LiDAR queue overflow");
            return;
        }
        PointCloudXYZI::Ptr points(new PointCloudXYZI);
        preprocess_.process(msg, points);
        // Use the same sampled scan-end convention as the baseline FAST-LIO.
        const double duration = points->size() > 1 ? points->back().curvature / 1000.0 : 0;
        if (duration > 0 && duration >= 0.5 * mean_duration_)
            mean_duration_ += (duration - mean_duration_) / ++duration_samples_;
        const double end = begin + (duration > 0 && duration >= 0.5 * mean_duration_ ? duration : mean_duration_);
        if (end <= last_scan_end_) {
            fail("FAILED", "derived scan-end timestamp reversed");
            return;
        }
        scans_.push_back({points, begin, end, received});
        last_scan_end_ = end;
        input_ms_ = last_lidar_stamp_ > 0 ? 1000 * (begin - last_lidar_stamp_) : 0;
        last_lidar_stamp_ = begin;
        const std::lock_guard<std::mutex> lock(health_);
        last_lidar_ = received;
        ++lidar_count_;
    }

    void imu(const sensor_msgs::Imu::ConstPtr &msg) {
        if (terminal()) return;
        try {
            check_finite(msg->linear_acceleration);
            check_finite(msg->angular_velocity);
            const double stamp = msg->header.stamp.toSec() - time_offset_;
            if (stamp <= last_imu_stamp_ || stamp <= 0) throw std::runtime_error("IMU timestamp repeated or reversed");
            auto adjusted = boost::make_shared<sensor_msgs::Imu>(*msg);
            adjusted->header.stamp.fromSec(stamp);
            imus_.push_back(adjusted);
            last_imu_stamp_ = stamp;
            // Prune pre-roll if only the IMU is active.
            if (scans_.empty() && last_lidar_stamp_ == 0) {
                while (imus_.size() > 100) imus_.pop_front();
            }
            if (imus_.size() > 400) throw std::runtime_error("IMU queue overflow");
            const std::lock_guard<std::mutex> lock(health_);
            last_imu_ = Clock::now();
        } catch (const std::exception &exc) {
            fail("FAILED", exc.what());
        }
    }

    void initial_transform(const geometry_msgs::TransformStamped::ConstPtr &msg) {
        if (tracking_ || pending_init_ || terminal()) return;
        const double age = (last_bootstrap_stamp_ - msg->header.stamp).toSec();
        if (msg->header.frame_id != kMap || msg->child_frame_id != kLocal ||
            last_bootstrap_stamp_.isZero() || age < -0.002 || age > kFresh) return;
        const auto &r = msg->transform.rotation;
        const auto &t = msg->transform.translation;
        Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
        if (!q.coeffs().allFinite() || std::abs(q.norm() - 1) > 0.02 ||
            !V3D(t.x, t.y, t.z).allFinite()) {
            fail("FAILED", "invalid initial transform");
            return;
        }
        map_from_local_.linear() = q.normalized().toRotationMatrix();
        map_from_local_.translation() = V3D(t.x, t.y, t.z);
        local_from_map_ = map_from_local_.inverse();
        pending_init_ = true;
    }

    void activate_prior() {
        // The fixed tree is already built in drone_map. Switch the index at
        // the scan boundary without changing the filter's bootstrap frame.
        tree_ = std::move(fixed_tree_);
        neighbors_.clear();
        tracking_ = true;
        pending_init_ = false;
    }

    void process_scan() {
        const auto started = Clock::now();
        const Scan scan = scans_.front();
        scans_.pop_front();
        const auto stamp = ros::Time().fromSec(scan.end);
        Stats frame;
        frame.processed = ++processed_count_;
        frame.input_ms = input_ms_;
        frame.queue_ms = elapsed(scan.received) * 1000;
        frame.backlog = scans_.size();
        MeasureGroup measure;
        measure.lidar = scan.cloud;
        measure.lidar_beg_time = scan.begin;
        measure.lidar_end_time = scan.end;
        while (!imus_.empty() && imus_.front()->header.stamp.toSec() <= scan.end) {
            measure.imu.push_back(imus_.front());
            imus_.pop_front();
        }
        if (processed_count_ == 1) {
            imu_.first_lidar_time = scan.begin;
            finish(frame, stamp, started, {}, {});
            return;
        }
        if (measure.imu.empty()) throw std::runtime_error("scan has no synchronized IMU");
        if (pending_init_) activate_prior();
        undistorted_->clear();
        imu_.Process(measure, filter_, undistorted_);
        if (undistorted_->empty()) {
            if (tracking_) throw std::runtime_error("no undistorted points");
            finish(frame, stamp, started, {}, {});
            return;
        }
        const auto predicted = filter_.get_x();
        if (!finite_state(predicted)) throw std::runtime_error("nonfinite predicted state");
        voxel_.setInputCloud(undistorted_);
        voxel_.filter(features_);
        const size_t count = features_.size();
        frame.points = count;
        if (count < NUM_MATCH_POINTS) {
            if (tracking_) throw std::runtime_error("no valid point-to-plane match: too few points");
            finish(frame, stamp, started, {}, {});
            return;
        }
        if (!tree_) {
            PointVector points;
            points.reserve(count);
            for (const auto &p : features_) points.push_back(world_point(predicted, p));
            tree_ = std::make_unique<KD_TREE<PointType>>();
            tree_->set_downsample_param(map_leaf_);
            tree_->Build(std::move(points));
            finish(frame, stamp, started, {}, {});
            return;
        }

        neighbors_.resize(count);
        selected_.assign(count, false);
        planes_.resize(count);
        double solve_time = 0;
        matches_ = 0;
        filter_.update_iterated_dyn_share_modified(kLaserVariance, solve_time);
        const auto state = filter_.get_x();
        frame.matches = matches_;
        frame.residual = mean_residual_;
        frame.map_points = tree_->validnum();
        if (!finite_state(state) || !filter_.get_P().allFinite())
            throw std::runtime_error("nonfinite updated state or covariance");
        if (matches_ == 0) {
            if (tracking_) {
                fail("FAILED", "no valid point-to-plane match");
                return;
            }
            finish(frame, stamp, started, {}, {});
            return;
        }
        frame.pos_update = (state.pos - predicted.pos).norm();
        frame.rot_update = Eigen::AngleAxisd(state.rot.toRotationMatrix() *
                                         predicted.rot.toRotationMatrix().transpose()).angle();
        if (!tracking_) add_bootstrap_points(state);
        auto out = odometry(state, stamp);
        sensor_msgs::PointCloud2::Ptr cloud;
        if (tracking_ && publish_cloud_ && registered_.getNumSubscribers()) {
            PointCloudXYZI points;
            points.reserve(undistorted_->size());
            for (const auto &p : *undistorted_) {
                auto point = world_point(state, p);
                point.getVector3fMap() = (map_from_local_ * point.getVector3fMap().cast<double>()).cast<float>();
                points.push_back(point);
            }
            cloud = boost::make_shared<sensor_msgs::PointCloud2>();
            pcl::toROSMsg(points, *cloud);
            cloud->header = out->header;
        }
        finish(frame, stamp, started, out, cloud);
    }

    static bool finite_state(const state_ikfom &s) {
        return s.pos.allFinite() && s.rot.coeffs().allFinite() && s.vel.allFinite() &&
               s.bg.allFinite() && s.ba.allFinite() && s.grav.vec.allFinite() &&
               s.offset_R_L_I.coeffs().allFinite() && s.offset_T_L_I.allFinite();
    }

    static PointType world_point(const state_ikfom &s, const PointType &p) {
        PointType result{};
        result.getVector3fMap() = (s.rot * (s.offset_R_L_I *
            p.getVector3fMap().cast<double>() + s.offset_T_L_I) + s.pos).cast<float>();
        result.intensity = p.intensity;
        return result;
    }

    // YWL h_share_model: same five-neighbor plane fit, score, and IEKF Jacobian.
    // Fixed extrinsics omit the six calibration columns. Sized vectors replace the upstream scratch arrays.
    static void observe(state_ikfom &s, esekfom::dyn_share_datastruct<double> &data) {
        auto &self = *active_node_;
        self.matches_ = 0;
        self.mean_residual_ = 0;
        for (size_t i = 0; i < self.features_.size(); ++i) {
            const auto &point = self.features_[i];
            const auto world = world_point(s, point);
            auto &near = self.neighbors_[i];
            if (data.converge || near.empty()) {
                std::vector<float> distance;
                auto query = world;
                if (self.tracking_) {
                    query.getVector3fMap() =
                        (self.map_from_local_ * world.getVector3fMap().cast<double>()).cast<float>();
                }
                self.tree_->Nearest_Search(query, NUM_MATCH_POINTS, near, distance);
                if (self.tracking_) {
                    // Keep the existing local-frame point-to-plane residual
                    // and Jacobian. A rigid transform preserves search distance.
                    for (auto &neighbor : near) {
                        const V3D local = self.local_from_map_ * neighbor.getVector3fMap().cast<double>();
                        neighbor.getVector3fMap() = local.cast<float>();
                    }
                }
                self.selected_[i] = near.size() >= NUM_MATCH_POINTS && distance[NUM_MATCH_POINTS - 1] <= 5;
            }
            if (!self.selected_[i]) continue;
            self.selected_[i] = false;
            VF(4) plane;
            if (!esti_plane(plane, near, 0.1f)) continue;
            const float residual = plane.head<3>().dot(world.getVector3fMap()) + plane[3];
            const float score = 1 - 0.9f * std::abs(residual) / std::sqrt(point.getVector3fMap().norm());
            if (!(score > 0.9f)) continue;
            plane[3] = residual;
            self.planes_[i] = plane;
            self.selected_[i] = true;
            self.mean_residual_ += std::abs(residual);
            ++self.matches_;
        }
        if (self.matches_ == 0) {
            data.valid = false;
            return;
        }
        self.mean_residual_ /= self.matches_;
        data.h_x = Eigen::MatrixXd::Zero(self.matches_, 12);
        data.h.resize(self.matches_);
        int row = 0;
        for (size_t i = 0; i < self.features_.size(); ++i) {
            if (!self.selected_[i]) continue;
            const auto &plane = self.planes_[i];
            const V3D normal = plane.head<3>().cast<double>();
            const V3D point = s.offset_R_L_I * self.features_[i].getVector3fMap().cast<double>() + s.offset_T_L_I;
            const V3D rotation_jacobian = point.cross(s.rot.conjugate() * normal);
            data.h_x.block<1, 6>(row, 0) << normal.transpose(), rotation_jacobian.transpose();
            data.h(row++) = -plane[3];
        }
    }

    void add_bootstrap_points(const state_ikfom &s) {
        // Bound the temporary map while waiting for the initial relocalization.
        if (tree_->validnum() > 150000) throw std::runtime_error("bootstrap map limit reached");
        PointVector downsampled, sparse;
        for (size_t i = 0; i < features_.size(); ++i) {
            const auto point = world_point(s, features_[i]);
            const auto &near = neighbors_[i];
            if (near.empty()) {
                downsampled.push_back(point);
                continue;
            }
            const Eigen::Array3f xyz = point.getVector3fMap().array();
            const Eigen::Array3f center = ((xyz / map_leaf_).floor() + 0.5f) * map_leaf_;
            if (((near.front().getArray3fMap() - center).abs() > 0.5f * map_leaf_).all()) {
                sparse.push_back(point);
                continue;
            }
            bool insert = true;
            if (near.size() >= NUM_MATCH_POINTS) {
                const float distance = (xyz - center).matrix().norm();
                for (int j = 0; j < NUM_MATCH_POINTS; ++j) {
                    if ((near[j].getArray3fMap() - center).matrix().norm() < distance) {
                        insert = false;
                        break;
                    }
                }
            }
            if (insert) downsampled.push_back(point);
        }
        tree_->Add_Points(downsampled, true);
        tree_->Add_Points(sparse, false);
    }

    nav_msgs::Odometry::Ptr odometry(const state_ikfom &s, const ros::Time &stamp) {
        const Pose reference = tracking_ ? map_from_local_ : Pose::Identity();
        const V3D position = reference * s.pos;
        const M3D rotation = reference.linear() * s.rot.toRotationMatrix();
        auto out = boost::make_shared<nav_msgs::Odometry>();
        out->header.stamp = stamp;
        out->header.frame_id = tracking_ ? kMap : kLocal;
        out->child_frame_id = kImu;
        out->pose.pose.position.x = position.x();
        out->pose.pose.position.y = position.y();
        out->pose.pose.position.z = position.z();
        out->pose.pose.orientation = quaternion(rotation);
        using Cov6 = Eigen::Matrix<double, 6, 6>;
        Cov6 jacobian = Cov6::Zero();
        jacobian.topLeftCorner<3, 3>() = reference.linear();
        jacobian.bottomRightCorner<3, 3>() = rotation;
        const Cov6 covariance = jacobian * filter_.get_P().topLeftCorner<6, 6>() * jacobian.transpose();
        const Cov6 symmetric = 0.5 * (covariance + covariance.transpose());
        if (!position.allFinite() || !symmetric.allFinite() || (symmetric.diagonal().array() < 0).any())
            throw std::runtime_error("invalid output pose or covariance");
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) out->pose.covariance[i*6+j] = symmetric(i,j);
        // Twist is not a supported output. Velocity is a stamped world vector.
        return out;
    }

    void finish(Stats frame, const ros::Time &stamp, Clock::time_point started,
                const nav_msgs::Odometry::Ptr &out, const sensor_msgs::PointCloud2::Ptr &cloud) {
        V3D velocity = V3D::Zero();
        if (out && tracking_) {
            velocity = map_from_local_.linear() * filter_.get_x().vel;
            if (!velocity.allFinite()) throw std::runtime_error("invalid map velocity");
        }
        const std::lock_guard<std::mutex> lock(health_);
        if (state_ == "STALE" || state_ == "FAILED") return;
        if (elapsed(last_lidar_) > kFresh || elapsed(last_imu_) > kFresh ||
            frame.queue_ms + elapsed(started) * 1000 > kFresh * 1000) {
            fail_locked("STALE", "stale input or scan processing deadline");
            return;
        }
        frame.lidar = lidar_count_;
        frame.published = published_count_;
        if (out) {
            if (tracking_) {
                geometry_msgs::Vector3Stamped vel;
                vel.header = out->header;
                vel.vector.x = velocity.x(); vel.vector.y = velocity.y(); vel.vector.z = velocity.z();
                state_ = "TRACKING";
                reason_ = "fixed prior";
                odom_.publish(out);
                velocity_.publish(vel);
                if (cloud) registered_.publish(cloud);
                frame.published = ++published_count_;
            } else {
                last_bootstrap_stamp_ = stamp;
                bootstrap_.publish(out);
            }
        }
        frame.work_ms = elapsed(started) * 1000;
        stats_ = frame;
        last_processed_ = Clock::now();
        status_stamp_ = stamp;
        diagnostic_locked();
    }

    void watchdog(const ros::SteadyTimerEvent &) {
        std::string problem;
        if (configured_) {
            try {
                if (file_version(pcd_) != pcd_version_ || file_version(meta_) != meta_version_)
                    problem = "prior map files changed";
                for (const auto &param : config_snapshot_) {
                    XmlRpc::XmlRpcValue current;
                    if (!pnh_.getParamCached(param.first, current) || current != param.second)
                        problem = "map or calibration parameter changed: " + param.first;
                }
            } catch (const std::exception &exc) {
                problem = exc.what();
            }
        }
        const std::lock_guard<std::mutex> lock(health_);
        if (!problem.empty()) fail_locked("FAILED", problem);
        if (state_ != "FAILED" && state_ != "STALE" && lidar_count_ > 0 &&
            (elapsed(last_lidar_) > kFresh || elapsed(last_imu_) > kFresh ||
             (stats_.processed > 0 && elapsed(last_processed_) > kFresh))) {
            fail_locked("STALE", "LiDAR, IMU or processing timeout");
        }
        if (elapsed(last_diagnostic_) > 0.25) diagnostic_locked();
    }

    void diagnostic_locked() {
        diagnostic_msgs::DiagnosticArray array;
        array.header.stamp = status_stamp_;
        diagnostic_msgs::DiagnosticStatus status;
        status.name = "ywl_localization";
        status.hardware_id = "MID360S";
        status.level = state_ == "TRACKING" ? status.OK :
            (state_ == "FAILED" ? status.ERROR : (state_ == "STALE" ? status.STALE : status.WARN));
        status.message = state_;
        auto kv = [&status](const std::string &key, const std::string &value) {
            diagnostic_msgs::KeyValue pair;
            pair.key = key; pair.value = value; status.values.push_back(pair);
        };
        kv("reason", reason_);
        kv("map_pcd_sha256", pcd_sha_);
        kv("map_meta_sha256", meta_sha_);
        kv("received_scans", std::to_string(lidar_count_));
        kv("processed_scans", std::to_string(stats_.processed));
        kv("published_scans", std::to_string(stats_.published));
        kv("scan_points", std::to_string(stats_.points));
        kv("matched_points", std::to_string(stats_.matches));
        kv("match_fraction", std::to_string(stats_.points ? double(stats_.matches) / stats_.points : 0.0));
        kv("residual_mean_m", std::to_string(stats_.residual));
        kv("map_points", std::to_string(stats_.map_points));
        kv("update_m", std::to_string(stats_.pos_update));
        kv("update_rad", std::to_string(stats_.rot_update));
        kv("processing_ms", std::to_string(stats_.work_ms));
        kv("receive_to_process_ms", std::to_string(stats_.queue_ms));
        kv("queue_depth", std::to_string(stats_.backlog));
        kv("lidar_interval_ms", std::to_string(stats_.input_ms));
        array.status.push_back(std::move(status));
        status_.publish(array);
        last_diagnostic_ = Clock::now();
    }

    static Node *active_node_;
    ros::NodeHandle nh_, pnh_, watchdog_nh_;
    ros::CallbackQueue watchdog_queue_;
    ros::SteadyTimer watch_;
    ros::Publisher bootstrap_, odom_, velocity_, registered_, status_;
    ros::Subscriber lidar_sub_, imu_sub_, init_sub_;
    std::mutex health_;
    std::string state_ = "WAITING_INIT", reason_ = "waiting for synchronized scans and confirmed initialization";
    std::string pcd_, meta_, pcd_sha_, meta_sha_;
    std::array<long long, 7> pcd_version_{}, meta_version_{};
    std::vector<std::pair<std::string, XmlRpc::XmlRpcValue>> config_snapshot_;
    Stats stats_;
    ros::Time status_stamp_, last_bootstrap_stamp_;
    Clock::time_point last_lidar_ = Clock::now(), last_imu_ = Clock::now();
    Clock::time_point last_processed_ = Clock::now(), last_diagnostic_ = Clock::now();
    uint64_t lidar_count_ = 0, processed_count_ = 0, published_count_ = 0;
    double last_lidar_stamp_ = 0, last_imu_stamp_ = 0, last_scan_end_ = 0;
    double time_offset_ = 0, map_leaf_ = 0.5, mean_duration_ = 0, input_ms_ = 0;
    unsigned duration_samples_ = 0;
    bool configured_ = false, pending_init_ = false, tracking_ = false, publish_cloud_ = false;
    Pose map_from_local_ = Pose::Identity(), local_from_map_ = Pose::Identity();
    V3D extrinsic_t_;
    M3D extrinsic_r_;
    Preprocess preprocess_;
    ImuProcess imu_;
    esekfom::esekf<state_ikfom, 12, input_ikfom> filter_;
    pcl::VoxelGrid<PointType> voxel_;
    PointCloudXYZI features_;
    PointCloudXYZI::Ptr undistorted_{new PointCloudXYZI};
    std::unique_ptr<KD_TREE<PointType>> tree_, fixed_tree_;
    std::deque<Scan> scans_;
    std::deque<sensor_msgs::Imu::ConstPtr> imus_;
    std::vector<PointVector> neighbors_;
    std::vector<bool> selected_;
    std::vector<VF(4), Eigen::aligned_allocator<VF(4)>> planes_;
    int matches_ = 0;
    double mean_residual_ = 0;
};

Node *Node::active_node_ = nullptr;
}  // namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "ywl_localization");
    Node node;
    node.run();
}
