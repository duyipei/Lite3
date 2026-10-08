
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <iostream>
#include <string>
#include <limits>
#include <flann/flann.hpp>

#include <rclcpp/rclcpp.hpp>
#include <pcl_ros/point_cloud.hpp>
#include <pcl_ros/transforms.hpp>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <pclomp/ndt_omp.h>
#include <fast_gicp/ndt/ndt_cuda.hpp>

#include <hdl_localization/pose_estimator.hpp>
#include <hdl_localization/delta_estimater.hpp>

#include <hdl_localization/msg/scan_matching_status.hpp>
#include <hdl_global_localization/srv/set_global_map.hpp>
#include <hdl_global_localization/srv/query_global_localization.hpp>

constexpr int print_ms_per_sec = 10;
using namespace std::chrono_literals;

namespace hdl_localization {

class HdlLocalizationApp : public rclcpp::Node{
public:
    using PointT = pcl::PointXYZI;
    HdlLocalizationApp(rclcpp::NodeOptions options) 
    : Node("hdl_localization", options), tf_buffer(get_clock()), tf_listener(tf_buffer), tf_broadcaster(*this){
        InitParams();
        CalLidar2BaseStaticTF();

        downsample_filter_.setLeafSize(downsample_resolution_, downsample_resolution_, downsample_resolution_);

        RCLCPP_INFO(get_logger(), "create registration method for localization");
        registration = CreateRegistration();

        // global localization
        RCLCPP_INFO(get_logger(), "create registration method for fallback during relocalization");
        relocalizing = false;
        delta_estimater.reset(new DeltaEstimater(CreateRegistration()));

        timer_ = create_wall_timer(
          std::chrono::seconds(print_ms_per_sec), 
          std::bind(&HdlLocalizationApp::TimerCallback, this));


        if(specify_init_pose){
            RCLCPP_INFO(get_logger(), "initialize pose estimator with specified parameters!!");
            pose_estimator.reset(
                new hdl_localization::PoseEstimator(
                    registration,
                    Eigen::Vector3f(init_pos_x, init_pos_y, init_pos_z),
                    Eigen::Quaternionf(init_ori_w, init_ori_x, init_ori_y, init_ori_z),
                    cool_time_duration
                )
            );
        }

        // LidarCallback always needs a time-aligned IMU sample to calculate
        // the base-to-footprint leveling transform.  Keep this subscription
        // active even when IMU-based pose prediction is disabled.  Previously
        // use_imu=false also disabled the subscription, so every lidar frame
        // was rejected by GetClosestImuData() before scan matching.
        if(use_imu){
            RCLCPP_INFO(get_logger(), "enable imu-based prediction");
        }else{
            RCLCPP_INFO(get_logger(), "disable imu-based pose prediction; keep IMU leveling enabled");
        }
        imu_sub = create_subscription<sensor_msgs::msg::Imu>(
            "/gpsimu_driver/imu_data",
            10,
            [this](sensor_msgs::msg::Imu::SharedPtr msg){
                this->ImuCallback(msg);
            }
        );
        auto reentrant_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions reentrant_options;
        reentrant_options.callback_group = reentrant_callback_group;
        points_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 
            1,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg){
                this->LidarCallback(msg);
            },
            reentrant_options
        );
        globalmap_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/globalmap", 
            1, 
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg){
                this->GlobalMapCallback(msg);
            }
        );
        initialpose_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 
            8, 
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
                this->InitialPoseCallback(msg);
            }
        );
        leg_odom2_sub = create_subscription<nav_msgs::msg::Odometry>(
            "/leg_odom2", 
            8,
            [this](nav_msgs::msg::Odometry::SharedPtr msg){
                this->LegOdom2Callback(msg);
            }
        );

        pose_pub = create_publisher<nav_msgs::msg::Odometry>("/odom", 5);
        aligned_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_points", 5);
        status_pub = create_publisher<msg::ScanMatchingStatus>("/status", 5);
        reanchor_pub = create_publisher<std_msgs::msg::String>(
            "/patrol/ndt_reanchor", rclcpp::QoS(1).reliable().transient_local());

        if(use_global_localization){        
            set_global_map_service = create_client<hdl_global_localization::srv::SetGlobalMap>("/hdl_global_localization/set_global_map");
            query_global_localization_service = create_client<hdl_global_localization::srv::QueryGlobalLocalization>("/hdl_global_localization/query");
            RCLCPP_INFO(get_logger(), "wait for global localization services");
            set_global_map_service->wait_for_service();
            query_global_localization_service->wait_for_service();
            relocalize_server = create_service<std_srvs::srv::Empty>(
                "/relocalize", 
                [this](std::shared_ptr<rmw_request_id_t> request_header,
                    std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                    std::shared_ptr<std_srvs::srv::Empty::Response> res)
                {
                    this->Relocalize(request_header, req, res);
                }
            );
        }
    }
    
    virtual ~HdlLocalizationApp(){

    }

private:
    // Same exact XYZ/L2 search and leaf size as PCL KdTreeFLANN, but one
    // FLANN call per cloud rather than thousands of parallel-region entries.
    std::vector<float> batch_target_xyz_;
    std::unique_ptr<flann::Index<flann::L2_Simple<float>>> batch_target_index_;
    pcl::PointCloud<PointT>::ConstPtr batch_target_cloud_;
    int batch_validation_remaining_ = 4;
    sensor_msgs::msg::PointCloud2::SharedPtr last_globalmap_message_;

    std::vector<float> TargetDistances(const pcl::PointCloud<PointT>& query) {
        auto target = registration->getInputTarget();
        if (batch_target_cloud_ != target) {
            batch_target_index_.reset();
            batch_target_xyz_.clear();
            for (const auto& p : *target) {
                if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
                    batch_target_xyz_.insert(batch_target_xyz_.end(), {p.x, p.y, p.z});
                }
            }
            if (!batch_target_xyz_.empty()) {
                flann::Matrix<float> dataset(batch_target_xyz_.data(), batch_target_xyz_.size()/3, 3);
                batch_target_index_.reset(new flann::Index<flann::L2_Simple<float>>(
                    dataset, flann::KDTreeSingleIndexParams(15)));
                batch_target_index_->buildIndex();
            }
            batch_target_cloud_ = target;
            batch_validation_remaining_ = 4;
        }
        std::vector<float> result(query.size(), std::numeric_limits<float>::infinity());
        if (!batch_target_index_ || query.empty()) return result;
        std::vector<float> xyz;
        std::vector<size_t> rows;
        xyz.reserve(query.size()*3);
        for (size_t i=0; i<query.size(); ++i) {
            const auto& p=query[i];
            if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
                xyz.insert(xyz.end(), {p.x,p.y,p.z}); rows.push_back(i);
            }
        }
        if (rows.empty()) return result;
        std::vector<int> indices(rows.size());
        std::vector<float> distances(rows.size());
        flann::Matrix<float> queries(xyz.data(), rows.size(), 3);
        flann::Matrix<int> neighbors(indices.data(), rows.size(), 1);
        flann::Matrix<float> squared(distances.data(), rows.size(), 1);
        flann::SearchParams params(-1, 0.0f);
        params.cores = 1;
        batch_target_index_->knnSearch(queries, neighbors, squared, 1, params);
        for (size_t i=0; i<rows.size(); ++i) result[rows[i]]=distances[i];
        // Verify the actual map/query against the original implementation on
        // startup. Fail closed on any disagreement; never weaken the gate.
        if (batch_validation_remaining_ > 0) {
            std::vector<int> old_indices(1);
            std::vector<float> old_distances(1);
            for (size_t i : rows) {
                const int count = registration->getSearchMethodTarget()->nearestKSearch(
                    query[i], 1, old_indices, old_distances);
                if (count != 1 || std::abs(old_distances[0]-result[i]) >
                    1e-5f * std::max(1.0f, old_distances[0])) {
                    throw std::runtime_error("batch nearest-neighbor validation failed");
                }
            }
            --batch_validation_remaining_;
            RCLCPP_INFO(get_logger(), "batch nearest-neighbor verified against PCL: %zu queries", rows.size());
        }
        return result;
    }

    double BatchFitnessScore(double max_range) {
        pcl::PointCloud<PointT> transformed;
        pcl::transformPointCloud(*registration->getInputSource(), transformed,
                                 registration->getFinalTransformation());
        const auto distances = TargetDistances(transformed);
        double sum=0.0; size_t count=0;
        // Preserve PCL's comparison to squared distance (not max_range^2).
        for (float d : distances) if (d <= max_range) { sum+=d; ++count; }
        return count ? sum/count : std::numeric_limits<double>::max();
    }

    pcl::Registration<PointT, PointT>::Ptr CreateRegistration(){
        if(reg_method == "NDT_OMP"){
            RCLCPP_INFO(get_logger(), "NDT_OMP is selected");
            pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
            ndt->setTransformationEpsilon(0.01);
            ndt->setResolution(ndt_resolution);
            if (ndt_neighbor_search_method == "DIRECT1") {
                RCLCPP_INFO(get_logger(), "search_method DIRECT1 is selected");
                ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
            } else if (ndt_neighbor_search_method == "DIRECT7") {
                RCLCPP_INFO(get_logger(), "search_method DIRECT7 is selected");
                ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
            } else {
                if (ndt_neighbor_search_method == "KDTREE") {
                RCLCPP_INFO(get_logger(), "search_method KDTREE is selected");
                } else {
                RCLCPP_WARN(get_logger(), "invalid search method was given");
                RCLCPP_WARN(get_logger(), "default method is selected (KDTREE)");
                }
                ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
            }
            return ndt;
        }
        else if(reg_method.find("NDT_CUDA") != std::string::npos){
            RCLCPP_INFO(get_logger(), "NDT_CUDA is selected");
            boost::shared_ptr<fast_gicp::NDTCuda<PointT, PointT>> ndt(new fast_gicp::NDTCuda<PointT, PointT>);
            ndt->setResolution(ndt_resolution);

            if(reg_method.find("D2D") != std::string::npos) {
                ndt->setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
            } else if (reg_method.find("P2D") != std::string::npos) {
                ndt->setDistanceMode(fast_gicp::NDTDistanceMode::P2D);
            }

            if (ndt_neighbor_search_method == "DIRECT1") {
                RCLCPP_INFO(get_logger(), "search_method DIRECT1 is selected");
                ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT1);
            } else if (ndt_neighbor_search_method == "DIRECT7") {
                RCLCPP_INFO(get_logger(), "search_method DIRECT7 is selected");
                ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
            } else if (ndt_neighbor_search_method == "DIRECT_RADIUS") {
                RCLCPP_INFO_STREAM(get_logger(), "search_method DIRECT_RADIUS is selected : " << ndt_neighbor_search_radius);
                ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT_RADIUS, ndt_neighbor_search_radius);
            } else {
                RCLCPP_WARN(get_logger(), "invalid search method was given");
            }
            return ndt;
        }
        else{
            RCLCPP_ERROR_STREAM(get_logger(), "unknown registration method:" << reg_method);
            return nullptr;
        }
    }

    void InitParams(){
        declare_parameter("enable_robot_odometry_prediction", false);
        if(!get_parameter("enable_robot_odometry_prediction", enable_robot_odometry_prediction)){
            RCLCPP_INFO(get_logger(), "get_parameter enable_robot_odometry_prediction failed");
        }
        declare_parameter("status_max_correspondence_dist", 0.5);
        if(get_parameter("status_max_correspondence_dist", max_correspondence_dist)){
            RCLCPP_INFO(get_logger(), "status_max_correspondence_dist is: %lf", max_correspondence_dist);
        }
        declare_parameter("status_max_valid_point_dist", 25.0);
        if(get_parameter("status_max_valid_point_dist", max_valid_point_dist)){
            RCLCPP_INFO(get_logger(), "status_max_valid_point_dist is: %lf", max_valid_point_dist);
        }
        declare_parameter("robot_odom_frame_id", "robot_odom");
        if(get_parameter("robot_odom_frame_id", robot_odom_frame_id)){
            RCLCPP_INFO(get_logger(), "robot_odom_frame_id is: %s", robot_odom_frame_id.c_str());
        }
        // Keep the robot odometry prediction TF in a separate tree.  The
        // localization result is map -> base_link, while the Lite3 leg
        // odometry bridge publishes robot_odom -> leg_base_link.  Reusing
        // base_link here would give that frame two TF parents.
        declare_parameter("robot_odom_child_frame_id", "base_link");
        if(get_parameter("robot_odom_child_frame_id", robot_odom_child_frame_id)){
            RCLCPP_INFO(get_logger(), "robot_odom_child_frame_id is: %s", robot_odom_child_frame_id.c_str());
        }
        declare_parameter("odom_child_frame_id", "base_link");
        if(get_parameter("odom_child_frame_id", odom_child_frame_id)){
            RCLCPP_INFO(get_logger(), "odom_child_frame_id is: %s", odom_child_frame_id.c_str());
        }
        declare_parameter("use_imu", true);
        if(get_parameter("use_imu", use_imu)){
            RCLCPP_INFO(get_logger(), "use_imu is: %d", use_imu);
        }
        declare_parameter("invert_acc", false);
        if(get_parameter("invert_acc", invert_acc)){
            RCLCPP_INFO(get_logger(), "invert_acc is: %d", invert_acc);
        }
        declare_parameter("invert_gyro", false);
        if(get_parameter("invert_gyro", invert_gyro)){
            RCLCPP_INFO(get_logger(), "invert_gyro is: %d", invert_gyro);
        }
        declare_parameter("t_diff", 0.2);
        if(get_parameter("t_diff", t_diff_)){
            RCLCPP_INFO(get_logger(), "t_diff is: %lf", t_diff_);
        }
        declare_parameter("downsample_resolution", 0.1);
        if(get_parameter("downsample_resolution", downsample_resolution_)){
            RCLCPP_INFO(get_logger(), "downsample_resolution is: %lf", downsample_resolution_);
        }
        // global localization
        declare_parameter("use_global_localization", true);
        if(get_parameter("use_global_localization", use_global_localization)){
            RCLCPP_INFO(get_logger(), "use_global_localization is: %d", use_global_localization);
        }
        declare_parameter("specify_init_pose", true);
        if(get_parameter("specify_init_pose", specify_init_pose)){
            RCLCPP_INFO(get_logger(), "specify_init_pose is: %d", specify_init_pose);
        }
        declare_parameter("init_pos_x", 0.0);
        if(get_parameter("init_pos_x", init_pos_x)){
            RCLCPP_INFO(get_logger(), "init_pos_x is: %lf", init_pos_x);
        }
        declare_parameter("init_pos_y", 0.0);
        if(get_parameter("init_pos_y", init_pos_y)){
            RCLCPP_INFO(get_logger(), "init_pos_y is: %lf", init_pos_y);
        }
        declare_parameter("init_pos_z", 0.0);
        if(get_parameter("init_pos_z", init_pos_z)){
            RCLCPP_INFO(get_logger(), "init_pos_z is: %lf", init_pos_z);
        }
        declare_parameter("init_ori_w", 1.0);
        if(get_parameter("init_ori_w", init_ori_w)){
            RCLCPP_INFO(get_logger(), "init_ori_w is: %lf", init_ori_w);
        }
        declare_parameter("init_ori_x", 0.0);
        if(get_parameter("init_ori_x", init_ori_x)){
            RCLCPP_INFO(get_logger(), "init_ori_x is: %lf", init_ori_x);
        }
        declare_parameter("init_ori_y", 0.0);
        if(get_parameter("init_ori_y", init_ori_y)){
            RCLCPP_INFO(get_logger(), "init_ori_y is: %lf", init_ori_y);
        }          
        declare_parameter("init_ori_z", 0.0);
        if(get_parameter("init_ori_z", init_ori_z)){
            RCLCPP_INFO(get_logger(), "init_ori_z is: %lf", init_ori_z);
        }        
        declare_parameter("cool_time_duration", 0.5);
        if(get_parameter("cool_time_duration", cool_time_duration)){
            RCLCPP_INFO(get_logger(), "cool_time_duration is: %lf", cool_time_duration);
        }
        declare_parameter("reg_method", "NDT_OMP");
        if(get_parameter("reg_method", reg_method)){
            RCLCPP_INFO(get_logger(), "reg_method is: %s", reg_method.c_str());
        }
        declare_parameter("ndt_neighbor_search_method", "DIRECT7");
        if(get_parameter("ndt_neighbor_search_method", ndt_neighbor_search_method)){
            RCLCPP_INFO(get_logger(), "ndt_neighbor_search_method is: %s", ndt_neighbor_search_method.c_str());
        }
        declare_parameter("ndt_neighbor_search_radius", 2.0);
        if(get_parameter("ndt_neighbor_search_radius", ndt_neighbor_search_radius)){
            RCLCPP_INFO(get_logger(), "ndt_neighbor_search_radius is: %lf", ndt_neighbor_search_radius);
        }
        declare_parameter("ndt_resolution", 1.0);
        if(get_parameter("ndt_resolution", ndt_resolution)){
            RCLCPP_INFO(get_logger(), "ndt_resolution is: %lf", ndt_resolution);
        }
        declare_parameter("manual_pose_translation_gate", 0.75);
        get_parameter("manual_pose_translation_gate", manual_pose_translation_gate);
        declare_parameter("manual_pose_yaw_gate", 0.436332313);
        get_parameter("manual_pose_yaw_gate", manual_pose_yaw_gate);
        declare_parameter("auto_reanchor_enabled", true);
        get_parameter("auto_reanchor_enabled", auto_reanchor_enabled);
        declare_parameter("auto_reanchor_samples", 30);
        get_parameter("auto_reanchor_samples", auto_reanchor_samples);
        declare_parameter("auto_reanchor_position_span", 0.20);
        get_parameter("auto_reanchor_position_span", auto_reanchor_position_span);
        declare_parameter("auto_reanchor_yaw_span", 0.043633231);
        get_parameter("auto_reanchor_yaw_span", auto_reanchor_yaw_span);
        declare_parameter("auto_reanchor_max_translation", 0.75);
        get_parameter("auto_reanchor_max_translation", auto_reanchor_max_translation);
        declare_parameter("auto_reanchor_max_yaw", 0.261799388);
        get_parameter("auto_reanchor_max_yaw", auto_reanchor_max_yaw);
        declare_parameter("auto_reanchor_max_fitness", 0.15);
        get_parameter("auto_reanchor_max_fitness", auto_reanchor_max_fitness);
        declare_parameter("manual_gate_violation_samples", 3);
        get_parameter("manual_gate_violation_samples", manual_gate_violation_samples);
        declare_parameter("ndt_fusion_gain", 0.05);
        get_parameter("ndt_fusion_gain", ndt_fusion_gain);
        declare_parameter("ndt_fusion_max_translation_step", 0.01);
        get_parameter("ndt_fusion_max_translation_step", ndt_fusion_max_translation_step);
        declare_parameter("ndt_fusion_max_yaw_step", 0.003490659);
        get_parameter("ndt_fusion_max_yaw_step", ndt_fusion_max_yaw_step);
    }

private:
    bool specify_init_pose;
    double downsample_resolution_;
    double init_pos_x;            
    double init_pos_y;            
    double init_pos_z;            
    double init_ori_w;            
    double init_ori_x;            
    double init_ori_y;              
    double init_ori_z;                
    double cool_time_duration;

    static Eigen::Matrix4f OdomPoseMatrix(const nav_msgs::msg::Odometry& message){
        const auto& p = message.pose.pose.position;
        const auto& q = message.pose.pose.orientation;
        Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
        matrix.block<3, 3>(0, 0) =
            Eigen::Quaternionf(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
        matrix.block<3, 1>(0, 3) = Eigen::Vector3f(p.x, p.y, p.z);
        return matrix;
    }

    bool ExpectedPoseFromLegOdom(Eigen::Matrix4f& expected){
        std::lock_guard<std::mutex> anchor_lock(manual_anchor_mutex_);
        if(!manual_anchor_valid_){
            return false;
        }
        std::lock_guard<std::mutex> leg_lock(leg_odom_mutex_);
        if(!leg_odom2_msg){
            return false;
        }
        const Eigen::Matrix4f current_leg = OdomPoseMatrix(*leg_odom2_msg);
        expected = manual_map_anchor_ * leg_odom_anchor_.inverse() * current_leg;
        return true;
    }

    static double PoseYaw(const Eigen::Matrix4f& pose){
        return std::atan2(pose(1, 0), pose(0, 0));
    }

    static double AngleDistance(double first, double second){
        return std::abs(std::atan2(std::sin(first - second),
                                  std::cos(first - second)));
    }

    void ClearAutoReanchorCandidates(){
        auto_reanchor_candidates_.clear();
    }

    bool TryPromoteStableNdtCandidate(
        const Eigen::Matrix4f& candidate,
        double translation_from_seed,
        double yaw_from_seed,
        double fitness){
        if(!auto_reanchor_pending_){
            return false;
        }
        const bool candidate_is_safe = registration->hasConverged() &&
            std::isfinite(fitness) && fitness <= auto_reanchor_max_fitness &&
            translation_from_seed <= auto_reanchor_max_translation &&
            yaw_from_seed <= auto_reanchor_max_yaw;
        if(!candidate_is_safe){
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "NDT acquisition rejected: converged=%d fitness=%.4f delta=%.3fm yaw=%.2fdeg (limits %.3f/%.2fm/%.1fdeg)",
                registration->hasConverged(), fitness, translation_from_seed,
                yaw_from_seed * 180.0 / M_PI, auto_reanchor_max_fitness,
                auto_reanchor_max_translation, auto_reanchor_max_yaw * 180.0 / M_PI);
            ClearAutoReanchorCandidates();
            return false;
        }

        auto_reanchor_candidates_.push_back({
            static_cast<double>(candidate(0, 3)),
            static_cast<double>(candidate(1, 3)),
            PoseYaw(candidate)});
        const std::size_t required = static_cast<std::size_t>(
            std::max(10, auto_reanchor_samples));
        while(auto_reanchor_candidates_.size() > required){
            auto_reanchor_candidates_.pop_front();
        }
        if(auto_reanchor_candidates_.size() < required){
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "NDT acquisition accumulating: %zu/%zu fitness=%.4f delta=%.3fm",
                auto_reanchor_candidates_.size(), required, fitness, translation_from_seed);
            return false;
        }

        double mean_x = 0.0;
        double mean_y = 0.0;
        double yaw_sin = 0.0;
        double yaw_cos = 0.0;
        for(const auto& pose : auto_reanchor_candidates_){
            mean_x += pose[0];
            mean_y += pose[1];
            yaw_sin += std::sin(pose[2]);
            yaw_cos += std::cos(pose[2]);
        }
        mean_x /= auto_reanchor_candidates_.size();
        mean_y /= auto_reanchor_candidates_.size();
        const double mean_yaw = std::atan2(yaw_sin, yaw_cos);
        double max_position_deviation = 0.0;
        double max_yaw_deviation = 0.0;
        for(const auto& pose : auto_reanchor_candidates_){
            max_position_deviation = std::max(
                max_position_deviation,
                std::hypot(pose[0] - mean_x, pose[1] - mean_y));
            max_yaw_deviation = std::max(
                max_yaw_deviation, AngleDistance(pose[2], mean_yaw));
        }
        if(max_position_deviation > auto_reanchor_position_span ||
           max_yaw_deviation > auto_reanchor_yaw_span){
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "NDT acquisition unstable: position_span=%.3fm yaw_span=%.2fdeg",
                max_position_deviation, max_yaw_deviation * 180.0 / M_PI);
            return false;
        }

        {
            std::lock_guard<std::mutex> anchor_lock(manual_anchor_mutex_);
            std::lock_guard<std::mutex> leg_lock(leg_odom_mutex_);
            if(!leg_odom2_msg){
                ClearAutoReanchorCandidates();
                return false;
            }
            manual_map_anchor_ = candidate;
            leg_odom_anchor_ = OdomPoseMatrix(*leg_odom2_msg);
            manual_anchor_valid_ = true;
        }
        auto_reanchor_pending_ = false;
        correction_rejected_ = false;
        consecutive_gate_violations_ = 0;
        ClearAutoReanchorCandidates();

        std_msgs::msg::String event;
        event.data = "AUTO_REANCHOR:x=" + std::to_string(candidate(0, 3)) +
            ",y=" + std::to_string(candidate(1, 3)) +
            ",yaw_deg=" + std::to_string(PoseYaw(candidate) * 180.0 / M_PI);
        reanchor_pub->publish(event);
        RCLCPP_WARN(
            get_logger(),
            "auto-promoted stable NDT candidate as anchor: x=%.3f y=%.3f yaw=%.1f deg, fitness=%.4f",
            candidate(0, 3), candidate(1, 3),
            PoseYaw(candidate) * 180.0 / M_PI, fitness);
        return true;
    }

    bool FuseNdtCorrectionWithLegOdom(
        const Eigen::Matrix4f& ndt_pose,
        Eigen::Matrix4f& fused_pose){
        std::lock_guard<std::mutex> anchor_lock(manual_anchor_mutex_);
        std::lock_guard<std::mutex> leg_lock(leg_odom_mutex_);
        if(!manual_anchor_valid_ || !leg_odom2_msg){
            return false;
        }

        const Eigen::Matrix4f current_leg = OdomPoseMatrix(*leg_odom2_msg);
        const Eigen::Matrix4f leg_relative =
            leg_odom_anchor_.inverse() * current_leg;
        const Eigen::Matrix4f expected_pose =
            manual_map_anchor_ * leg_relative;

        // NDT can move by several decimetres between adjacent scans even
        // while the robot is stationary.  Keep leg odometry as the
        // continuous motion source and feed the NDT map correction into the
        // map anchor gradually.  This preserves global correction without
        // exposing scan-to-scan NDT jitter to Nav2 as robot motion.
        Eigen::Vector2f correction = static_cast<float>(ndt_fusion_gain) *
            (ndt_pose.block<2, 1>(0, 3) -
             expected_pose.block<2, 1>(0, 3));
        const double correction_norm = correction.norm();
        if(correction_norm > ndt_fusion_max_translation_step &&
           correction_norm > 0.0){
            correction *= static_cast<float>(
                ndt_fusion_max_translation_step / correction_norm);
        }

        const double signed_yaw_error = std::atan2(
            std::sin(PoseYaw(ndt_pose) - PoseYaw(expected_pose)),
            std::cos(PoseYaw(ndt_pose) - PoseYaw(expected_pose)));
        double yaw_correction = ndt_fusion_gain * signed_yaw_error;
        yaw_correction = std::max(
            -ndt_fusion_max_yaw_step,
            std::min(ndt_fusion_max_yaw_step, yaw_correction));

        fused_pose = expected_pose;
        fused_pose(0, 3) += correction.x();
        fused_pose(1, 3) += correction.y();
        fused_pose.block<3, 3>(0, 0) =
            Eigen::AngleAxisf(
                static_cast<float>(yaw_correction),
                Eigen::Vector3f::UnitZ()).toRotationMatrix() *
            expected_pose.block<3, 3>(0, 0);

        // Choose the anchor that reproduces the fused pose for the current
        // leg-odometry sample.  The leg anchor stays fixed, so subsequent
        // robot motion remains continuous.
        manual_map_anchor_ = fused_pose * leg_relative.inverse();
        return true;
    }

    void CalLidar2BaseStaticTF(){
        std::string lidar_frame = "rslidar";
        while(rclcpp::ok() && !tf_buffer.canTransform(odom_child_frame_id, lidar_frame, rclcpp::Clock(RCL_STEADY_TIME).now()) ){
            RCLCPP_WARN(get_logger(), "Waiting for transform from %s to %s", lidar_frame.c_str(), odom_child_frame_id.c_str());
            sleep(1);
        }
        geometry_msgs::msg::TransformStamped lidar2base_transform_ = tf_buffer.lookupTransform(odom_child_frame_id, lidar_frame, rclcpp::Time(3, 0));
        Eigen::Quaternionf rotation(lidar2base_transform_.transform.rotation.w,
                                    lidar2base_transform_.transform.rotation.x,
                                    lidar2base_transform_.transform.rotation.y,
                                    lidar2base_transform_.transform.rotation.z);
        Eigen::Vector3f translation(lidar2base_transform_.transform.translation.x,
                                    lidar2base_transform_.transform.translation.y,
                                    lidar2base_transform_.transform.translation.z);
        lidar2base_transform_matrix_.setIdentity();
        lidar2base_transform_matrix_.block<3, 3>(0, 0) = rotation.toRotationMatrix();
        lidar2base_transform_matrix_.block<3, 1>(0, 3) = translation;

        #ifdef DEBUG_TF
        std::cout << "w,x,y,z: " << lidar2base_transform_.transform.rotation.w << " "
            << lidar2base_transform_.transform.rotation.x << " "
            << lidar2base_transform_.transform.rotation.y << " "
            << lidar2base_transform_.transform.rotation.z << std::endl;
        std::cout << "rotation.toRotationMatrix()" << rotation.toRotationMatrix() << std::endl;
        std::cout << "x,y,z: " << lidar2base_transform_.transform.translation.x << " "
                    << lidar2base_transform_.transform.translation.y << " "
                    << lidar2base_transform_.transform.translation.z << " " << std::endl;
        std::cout << "translation" << translation << std::endl;
        std::cout << "camera2base_transform_matrix_: " << camera2base_transform_matrix_ << std::endl;
        #endif
    }

    void CalculateBase2FootPrintTF(sensor_msgs::msg::Imu::SharedPtr msg, Eigen::Matrix4f &base2footprint_transform_matrix){
        tf2::Quaternion quat;
        tf2::fromMsg(msg->orientation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf2::Quaternion(quat)).getRPY(roll, pitch, yaw);
        quat.setRPY(roll, pitch, 0.0);
        Eigen::Quaternionf eigen_quat(quat.w(), quat.x(), quat.y(), quat.z());
        base2footprint_transform_matrix = Eigen::Matrix4f::Identity();
        base2footprint_transform_matrix.block<3, 3>(0, 0) = eigen_quat.toRotationMatrix();
        base2footprint_transform_matrix(0, 3) = 0;
        base2footprint_transform_matrix(1, 3) = 0;
        base2footprint_transform_matrix(2, 3) = 0.34751;

        #ifdef DEBUG_TF
        std::cout << "rpy: " << roll << pitch << yaw << std::endl;
        std::cout << "base2footprint_transform_matrix: " << base2footprint_transform_matrix << std::endl;
        #endif
    }

    bool GetClosestImuData(const rclcpp::Time &time, sensor_msgs::msg::Imu::SharedPtr &msg){
        std::lock_guard<std::mutex> lock(imu_data_mutex);

        if(imu_data_buffer_.empty()){
            return false;
        }

        int index = 0;
        auto &stamp_ref = imu_data_buffer_.front()->header.stamp;
        double min_time_diff = std::abs(((double)stamp_ref.sec + (double)stamp_ref.nanosec/1e9) - time.seconds());
        for(int i = 0; i < imu_data_buffer_.size(); i++){
            auto &stamp_ref_i = imu_data_buffer_.at(i)->header.stamp;
            double time_diff = std::abs(((double)stamp_ref_i.sec + (double)stamp_ref_i.nanosec/1e9) - time.seconds());
            if(time_diff < min_time_diff){
                min_time_diff = time_diff;
                index = i;
            }
        }

        if(index > 0){
            for(int i = 0; i < index; i++){
                imu_data_buffer_.pop_front();
            }
        }

        msg = imu_data_buffer_.front();
        imu_data_buffer_.pop_front();
        return true;
    }

    Eigen::Matrix4f lidar2base_transform_matrix_;

    /**
     * @brief callback for imu data
     * @param imu_msg
     */
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
        std::lock_guard<std::mutex> lock(imu_data_mutex);
        imu_data_buffer_.push_back(imu_msg);
    }

    bool IsLidarTimeout(){
        rclcpp::Time now = rclcpp::Clock(RCL_STEADY_TIME).now();
        double t_diff = (now - last_lidar_time_).seconds();
        if(last_lidar_time_.nanoseconds() == 0){
            last_lidar_time_ = now;
            RCLCPP_WARN(get_logger(), "received first lidar");            
            return true;            
        }
        else if(t_diff >= t_diff_){
            last_lidar_time_ = now;
            // RCLCPP_WARN(get_logger(), "received timeout lidar, %lf", t_diff);
            return true;
        }
        else{
            last_lidar_time_ = now;
            return false;
        }
    }

    bool IsGlobalMapReceived(){
        if(!globalmap) {
            RCLCPP_ERROR(get_logger(), "globalmap has not been received!!");
            return false;
        }
        else{
            return true;
        }
    }

    /**
     * @brief callback for point cloud data
     * @param points_msg
     */
    void LidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        auto start = std::chrono::steady_clock::now();

        // data prepare
        if(IsLidarTimeout()){
            // Callback spacing includes the preceding scan's processing time.
            // Dropping the newly arrived scan cannot detect a sensor outage
            // and worsens overload. Keep the newest scan (DDS depth 1); the
            // independent status/odometry watchdog supervises actual liveness.
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "lidar callback gap exceeds %.3fs; processing newest scan", t_diff_);
        }
        if(!IsGlobalMapReceived()){
            RCLCPP_WARN(get_logger(), "global map not received!");
            return;
        }
        const auto& stamp = msg->header.stamp;
        rclcpp::Time points_time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_STEADY_TIME);
        sensor_msgs::msg::Imu::SharedPtr imu_data;
        if(!GetClosestImuData(msg->header.stamp, imu_data)){
            RCLCPP_WARN(get_logger(), "imu_data_buffer is empty");
            return;
        };
        Eigen::Matrix4f base2footprint_transform_matrix;
        CalculateBase2FootPrintTF(imu_data, base2footprint_transform_matrix);
        pcl_ros::transformPointCloud(lidar2base_transform_matrix_, *msg, *msg);
        pcl_ros::transformPointCloud(base2footprint_transform_matrix, *msg, *msg);
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::fromROSMsg(*msg, *cloud);
        auto filtered = Downsample(cloud);
        const auto prepared_at = std::chrono::steady_clock::now();
        last_scan = filtered;
        if(relocalizing) {
            delta_estimater->add_frame(filtered);
        }

        std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
        if(!pose_estimator){
            RCLCPP_ERROR(get_logger(), "waiting for initial pose input!!");
            return;
        }

        // predict
        if(!use_imu) {
            pose_estimator->predict(points_time);
        }else{
            rclcpp::Time imu_time(imu_data->header.stamp.sec, imu_data->header.stamp.nanosec, RCL_STEADY_TIME);
            const auto& acc = imu_data->linear_acceleration;
            const auto& gyro = imu_data->angular_velocity;
            double acc_sign = invert_acc ? -1.0 : 1.0;
            double gyro_sign = invert_gyro ? -1.0 : 1.0;
            pose_estimator->predict(imu_time, acc_sign * Eigen::Vector3f(acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
        }

        // odometry-based prediction
        rclcpp::Time last_correction_time = pose_estimator->last_correction_time();
        if(enable_robot_odometry_prediction && !last_correction_time.nanoseconds() == 0){
            geometry_msgs::msg::TransformStamped odom_delta;
            // Never block scan matching for a future odometry sample. Use the
            // existing latest-TF fallback immediately if exact-time TF is not
            // available; freshness is supervised independently.
            if(tf_buffer.canTransform(robot_odom_child_frame_id, last_correction_time, robot_odom_child_frame_id, rclcpp::Time(stamp.sec, stamp.nanosec), robot_odom_frame_id, rclcpp::Duration(0))){
                odom_delta = tf_buffer.lookupTransform(robot_odom_child_frame_id, last_correction_time, robot_odom_child_frame_id, stamp, robot_odom_frame_id, rclcpp::Duration(0));
            } 
            else if(tf_buffer.canTransform(robot_odom_child_frame_id, last_correction_time, robot_odom_child_frame_id, rclcpp::Time(0,0), robot_odom_frame_id, rclcpp::Duration(0))){
                odom_delta = tf_buffer.lookupTransform(robot_odom_child_frame_id, last_correction_time, robot_odom_child_frame_id, rclcpp::Time(0,0), robot_odom_frame_id, rclcpp::Duration(0));
            }

            if(odom_delta.header.stamp.nanosec == 0){
                RCLCPP_WARN_STREAM(get_logger(), "failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id);
            } 
            else{
                Eigen::Isometry3d delta = tf2::transformToEigen(odom_delta);
                pose_estimator->predict_odom(delta.cast<float>().matrix());
            }
        }

        // First refine the coarse manual/visual seed internally while keeping
        // the externally published pose pinned to the safe leg-odometry
        // prediction.  A candidate is promoted exactly once only after a
        // bounded, good-fitness cluster remains stable for many scans.
        const bool was_rejected = correction_rejected_;
        correction_rejected_ = false;
        auto aligned = pose_estimator->correct(points_time, filtered);
        const auto matched_at = std::chrono::steady_clock::now();
        Eigen::Matrix4f published_pose = pose_estimator->matrix();
        Eigen::Matrix4f expected_pose;
        if(ExpectedPoseFromLegOdom(expected_pose)){
            const Eigen::Matrix4f estimated_pose = pose_estimator->matrix();
            const double translation_error =
                (estimated_pose.block<2, 1>(0, 3) -
                 expected_pose.block<2, 1>(0, 3)).norm();
            const double estimated_yaw = PoseYaw(estimated_pose);
            const double expected_yaw = PoseYaw(expected_pose);
            const double yaw_error = AngleDistance(estimated_yaw, expected_yaw);
            const double fitness = BatchFitnessScore(
                max_correspondence_dist);

            if(auto_reanchor_enabled && auto_reanchor_pending_){
                const bool promoted = TryPromoteStableNdtCandidate(
                    estimated_pose, translation_error, yaw_error, fitness);
                if(!promoted){
                    correction_rejected_ = true;
                    published_pose = expected_pose;
                    pcl::transformPointCloud(*filtered, *aligned, expected_pose);

                    // A candidate outside the tightly bounded acquisition
                    // region is not allowed to keep walking through the map.
                    // Reset only unsafe candidates; bounded candidates may
                    // continue converging internally until their window is
                    // stable enough to promote.
                    if(!registration->hasConverged() || !std::isfinite(fitness) ||
                       fitness > auto_reanchor_max_fitness ||
                       translation_error > auto_reanchor_max_translation ||
                       yaw_error > auto_reanchor_max_yaw){
                        const Eigen::Quaternionf expected_quaternion(
                            expected_pose.block<3, 3>(0, 0));
                        pose_estimator.reset(
                            new hdl_localization::PoseEstimator(
                                registration,
                                expected_pose.block<3, 1>(0, 3),
                                expected_quaternion,
                                cool_time_duration));
                    }
                }else{
                    published_pose = estimated_pose;
                }
            }else if(!registration->hasConverged() ||
                     !std::isfinite(fitness) ||
                     fitness > auto_reanchor_max_fitness ||
                     translation_error > manual_pose_translation_gate ||
                     yaw_error > manual_pose_yaw_gate){
                correction_rejected_ = true;
                published_pose = expected_pose;
                pcl::transformPointCloud(*filtered, *aligned, expected_pose);
                consecutive_gate_violations_++;
                if(consecutive_gate_violations_ >=
                   std::max(1, manual_gate_violation_samples)){
                    const Eigen::Quaternionf expected_quaternion(
                        expected_pose.block<3, 3>(0, 0));
                    pose_estimator.reset(
                        new hdl_localization::PoseEstimator(
                            registration,
                            expected_pose.block<3, 1>(0, 3),
                            expected_quaternion,
                            cool_time_duration));
                    consecutive_gate_violations_ = 0;
                }
                if(!was_rejected){
                    RCLCPP_ERROR(
                        get_logger(),
                        "rejecting NDT correction: disagreement %.3f m, %.1f deg, fitness %.4f, converged=%s",
                        translation_error, yaw_error * 180.0 / M_PI,
                        fitness, registration->hasConverged() ? "true" : "false");
                }
            }else{
                consecutive_gate_violations_ = 0;
                if(!FuseNdtCorrectionWithLegOdom(
                       estimated_pose, published_pose)){
                    published_pose = expected_pose;
                }
                pcl::transformPointCloud(
                    *filtered, *aligned, published_pose);
                if(was_rejected){
                    RCLCPP_INFO(get_logger(), "NDT correction returned inside manual/leg anchor gate");
                }
            }
        }

        const auto gated_at = std::chrono::steady_clock::now();
        // pub
        if(aligned_pub->get_subscription_count()) {
            sensor_msgs::msg::PointCloud2 ros_cloud;
            pcl::toROSMsg(*aligned, ros_cloud);
            ros_cloud.header.frame_id = "map";
            ros_cloud.header.stamp = points_time;
            aligned_pub->publish(ros_cloud);
        }
        if(status_pub->get_subscription_count()) {
            PublishScanMatchingStatus(msg->header, aligned);
        }
        const auto status_at = std::chrono::steady_clock::now();
        PublishOdometry(msg->header.stamp, published_pose);

        auto end = std::chrono::steady_clock::now();
        auto diff = end - start;
        double ms = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(diff).count());
        const auto elapsed_ms = [](const auto& a, const auto& b) {
            return std::chrono::duration<double, std::milli>(b - a).count();
        };
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "scan profile points=%zu prep=%.1f match=%.1f gate=%.1f status=%.1f odom=%.1f total=%.1f ms",
            filtered->size(), elapsed_ms(start, prepared_at),
            elapsed_ms(prepared_at, matched_at), elapsed_ms(matched_at, gated_at),
            elapsed_ms(gated_at, status_at), elapsed_ms(status_at, end), ms);

        {
            std::lock_guard<std::mutex> lock(timer_data_mutex_);
            ms_deque_.push_back(ms);
            fps_count_++;
        }
    }

    /**
     * @brief callback for globalmap input
     * @param points_msg
     */
    void GlobalMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr points_msg) {
        // The server periodically republishes its map. A timestamp change alone
        // must not rebuild NDT and nearest-neighbor indices. Real map changes
        // (including layout and frame changes) still take the normal update path.
        if (last_globalmap_message_) {
            const auto& previous = *last_globalmap_message_;
            if (previous.header.frame_id == points_msg->header.frame_id &&
                previous.height == points_msg->height && previous.width == points_msg->width &&
                previous.fields == points_msg->fields &&
                previous.is_bigendian == points_msg->is_bigendian &&
                previous.point_step == points_msg->point_step &&
                previous.row_step == points_msg->row_step &&
                previous.is_dense == points_msg->is_dense && previous.data == points_msg->data) {
                return;
            }
        }
        // RCLCPP_INFO(get_logger(), "globalmap received!");
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::fromROSMsg(*points_msg, *cloud);
        globalmap = cloud;

        registration->setInputTarget(globalmap);
        last_globalmap_message_ = points_msg;

        if(use_global_localization) {
            RCLCPP_INFO(get_logger(), "set globalmap for global localization!");
            auto request = std::make_shared<hdl_global_localization::srv::SetGlobalMap::Request>();
            pcl::toROSMsg(*globalmap, request->global_map);
            auto result_future = set_global_map_service->async_send_request(request);
            if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(get_logger(), "failed to set global map");
            }
            else{
                RCLCPP_INFO(get_logger(), "done");
            }
        }
    }

    /**
     * @brief perform global localization to relocalize the sensor position
     * @param
     */
    bool Relocalize(
        std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<std_srvs::srv::Empty::Request> req, 
        std::shared_ptr<std_srvs::srv::Empty::Response> res) 
    {
        (void)request_header;
        if(last_scan == nullptr) {
            RCLCPP_INFO(get_logger(), ("no scan has been received"));
            return false;
        }

        relocalizing = true;
        delta_estimater->reset();
        pcl::PointCloud<PointT>::ConstPtr scan = last_scan;

        auto request = std::make_shared<hdl_global_localization::srv::QueryGlobalLocalization::Request>();
        pcl::toROSMsg(*scan, request->cloud);
        request->max_num_candidates = 1;
        auto result_future = query_global_localization_service->async_send_request(request);
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "failed to set global map");
            relocalizing = false;
            return false;
        }
        auto results = result_future.get();
        if(results->poses.empty()){
            RCLCPP_INFO(get_logger(), "failed to set global map");
            relocalizing = false;
            return false;
        }       
        
        const auto& result = results->poses[0];
        RCLCPP_INFO(get_logger(), "--- Global localization result ---");
        RCLCPP_INFO_STREAM(get_logger(), "Trans : " << result.position.x << " " << result.position.y << " " << result.position.z);
        RCLCPP_INFO_STREAM(get_logger(), "Quat  : " << result.orientation.x << " " << result.orientation.y << " " << result.orientation.z << " " << result.orientation.w);
        RCLCPP_INFO_STREAM(get_logger(), "Error : " << results->errors[0]);
        RCLCPP_INFO_STREAM(get_logger(), "Inlier: " << results->inlier_fractions[0]);

        Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
        pose.linear() = Eigen::Quaternionf(result.orientation.w, result.orientation.x, result.orientation.y, result.orientation.z).toRotationMatrix();
        pose.translation() = Eigen::Vector3f(result.position.x, result.position.y, result.position.z);
        pose = pose * delta_estimater->estimated_delta();

        double cool_time_duration;
        if(get_parameter("cool_time_duration", cool_time_duration)){
            RCLCPP_INFO(get_logger(), "cool_time_duration is: %lf", cool_time_duration);
        }

        {
            std::lock_guard<std::mutex> lock(pose_estimator_mutex);
            pose_estimator.reset(
                new hdl_localization::PoseEstimator(
                    registration,
                    pose.translation(),
                    Eigen::Quaternionf(pose.linear()),
                    cool_time_duration
                )
            );       
        }

        relocalizing = false;

        return true;
    }

    /**
     * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
     * @param pose_msg
     */
    void InitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg){
        RCLCPP_INFO(get_logger(), "initial pose received!!");
        const auto& p = pose_msg->pose.pose.position;
        const auto& q = pose_msg->pose.pose.orientation;
        double cool_time_duration;        
        if(get_parameter("cool_time_duration", cool_time_duration)){
            RCLCPP_INFO(get_logger(), "cool_time_duration is: %lf", cool_time_duration);
        }
        {
            std::lock_guard<std::mutex> lock(pose_estimator_mutex);
            pose_estimator.reset(
                new hdl_localization::PoseEstimator(
                    registration,
                    Eigen::Vector3f(p.x, p.y, p.z),
                    Eigen::Quaternionf(q.w, q.x, q.y, q.z),
                    cool_time_duration
                )
            );
            std::lock_guard<std::mutex> anchor_lock(manual_anchor_mutex_);
            std::lock_guard<std::mutex> leg_lock(leg_odom_mutex_);
            if(leg_odom2_msg){
                manual_map_anchor_.setIdentity();
                manual_map_anchor_.block<3, 3>(0, 0) =
                    Eigen::Quaternionf(q.w, q.x, q.y, q.z)
                        .normalized().toRotationMatrix();
                manual_map_anchor_.block<3, 1>(0, 3) =
                    Eigen::Vector3f(p.x, p.y, p.z);
                leg_odom_anchor_ = OdomPoseMatrix(*leg_odom2_msg);
                manual_anchor_valid_ = true;
                auto_reanchor_pending_ = auto_reanchor_enabled;
                ClearAutoReanchorCandidates();
                consecutive_gate_violations_ = 0;
                correction_rejected_ = auto_reanchor_pending_;
                RCLCPP_INFO(
                    get_logger(),
                    "manual pose anchored to current /leg_odom2 pose; stable NDT auto-reanchor %s",
                    auto_reanchor_pending_ ? "pending" : "disabled");
            }else{
                manual_anchor_valid_ = false;
                auto_reanchor_pending_ = false;
                ClearAutoReanchorCandidates();
                RCLCPP_ERROR(get_logger(), "cannot anchor manual pose: /leg_odom2 is unavailable");
            }
        }
    }

    void LegOdom2Callback(nav_msgs::msg::Odometry::SharedPtr msg){
        std::lock_guard<std::mutex> lock(leg_odom_mutex_);
        leg_odom2_msg = msg;
    }

    /**
     * @brief downsampling
     * @param cloud   input cloud
     * @return downsampled cloud
     */
    pcl::PointCloud<PointT>::ConstPtr Downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud){
        pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
        downsample_filter_.setInputCloud(cloud);
        downsample_filter_.filter(*filtered);
        filtered->header = cloud->header;
        return filtered;
    }

    /**
     * @brief publish odometry
     * @param stamp  timestamp
     * @param pose   odometry pose to be published
     */
    void PublishOdometry(const rclcpp::Time& stamp, const Eigen::Matrix4f& pose) {
        // broadcast the transform over tf
        // if(tf_buffer.canTransform(robot_odom_frame_id, odom_child_frame_id, rclcpp::Time(0))){
        //     geometry_msgs::msg::TransformStamped map_wrt_frame = tf2::eigenToTransform(Eigen::Isometry3d(pose.inverse().cast<double>()));
        //     map_wrt_frame.header.stamp = stamp;
        //     map_wrt_frame.header.frame_id = odom_child_frame_id;
        //     map_wrt_frame.child_frame_id = "map";

        //     geometry_msgs::msg::TransformStamped frame_wrt_odom = tf_buffer.lookupTransform(robot_odom_frame_id, odom_child_frame_id, rclcpp::Time(0, 0), rclcpp::Duration(0, 100000000));
        //     // Eigen::Matrix4f frame2odom = tf2::transformToEigen(frame_wrt_odom).cast<float>().matrix();

        //     geometry_msgs::msg::TransformStamped map_wrt_odom;
        //     tf2::doTransform(map_wrt_frame, map_wrt_odom, frame_wrt_odom);

        //     tf2::Transform odom_wrt_map;
        //     tf2::fromMsg(map_wrt_odom.transform, odom_wrt_map);
        //     odom_wrt_map = odom_wrt_map.inverse();

        //     geometry_msgs::msg::TransformStamped odom_trans;
        //     odom_trans.transform = tf2::toMsg(odom_wrt_map);
        //     odom_trans.header.stamp = stamp;
        //     odom_trans.header.frame_id = "map";
        //     odom_trans.child_frame_id = robot_odom_frame_id;

        //     tf_broadcaster.sendTransform(odom_trans);

        // } 
        // else{
            auto temp_pose1 = tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()));
            temp_pose1.transform.translation.z = 0.0;

            // RCLCPP_INFO(get_logger(), "localization: x:%lf, y:%lf", temp_pose1.transform.translation.x, temp_pose1.transform.translation.y);

            geometry_msgs::msg::TransformStamped odom_trans = temp_pose1;
            odom_trans.header.stamp = stamp;
            odom_trans.header.frame_id = "map";
            odom_trans.child_frame_id = odom_child_frame_id;
            tf_broadcaster.sendTransform(odom_trans);

            // geometry_msgs::msg::TransformStamped odom_trans;
            // odom_trans.header.stamp = stamp;
            // odom_trans.header.frame_id = "map";
            // odom_trans.child_frame_id = odom_child_frame_id;
            // odom_trans.transform.translation.x = 0;
            // odom_trans.transform.translation.y = 0;
            // odom_trans.transform.translation.z = 0;
            // odom_trans.transform.rotation.w = 1;
            // odom_trans.transform.rotation.x = 0;
            // odom_trans.transform.rotation.y = 0;
            // odom_trans.transform.rotation.z = 0;
            // tf_broadcaster.sendTransform(odom_trans);

        // }

        // publish the transform
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "map";

        auto temp_pose2 = tf2::toMsg(Eigen::Isometry3d(pose.cast<double>()));
        temp_pose2.position.z = 0.0;
        odom.pose.pose = temp_pose2;
        odom.child_frame_id = odom_child_frame_id;
        odom.twist.twist.linear.x = leg_odom2_msg->twist.twist.linear.x;
        odom.twist.twist.linear.y = leg_odom2_msg->twist.twist.linear.y;
        odom.twist.twist.angular.z = leg_odom2_msg->twist.twist.angular.z;

        pose_pub->publish(odom);
    }

    /**
     * @brief publish scan matching status information
     */
    void PublishScanMatchingStatus(const std_msgs::msg::Header& header, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned) {
        msg::ScanMatchingStatus status;
        status.header = header;

        status.has_converged = registration->hasConverged() && !correction_rejected_;
        status.matching_error = 0.0;
        
        int num_inliers = 0;
        int num_valid_points = 0;
        const auto squared_distances = TargetDistances(*aligned);
        for(int i = 0; i < static_cast<int>(aligned->size()); i++) {
            const auto& pt = aligned->at(i);
            if (pt.getVector3fMap().norm() > max_valid_point_dist) {
                continue;
            }
            num_valid_points++;

            if(squared_distances[i] < max_correspondence_dist * max_correspondence_dist) {
                status.matching_error += squared_distances[i];
                num_inliers++;
            }
        }

        status.matching_error = num_inliers ? status.matching_error / num_inliers :
            std::numeric_limits<float>::infinity();
        if (!num_inliers) status.has_converged = false;
        status.inlier_fraction = static_cast<float>(num_inliers) / std::max(1, num_valid_points);
        status.relative_pose = tf2::eigenToTransform(Eigen::Isometry3d(registration->getFinalTransformation().cast<double>())).transform;

        status.prediction_labels.reserve(2);
        status.prediction_errors.reserve(2);

        std::vector<double> errors(6, 0.0);

        if(pose_estimator->wo_prediction_error()) {
            status.prediction_labels.push_back(std_msgs::msg::String());
            status.prediction_labels.back().data = "without_pred";
            status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->wo_prediction_error().get().cast<double>())).transform);
        }

        if(pose_estimator->imu_prediction_error()) {
            status.prediction_labels.push_back(std_msgs::msg::String());
            status.prediction_labels.back().data = use_imu ? "imu" : "motion_model";
            status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->imu_prediction_error().get().cast<double>())).transform);
        }

        if(pose_estimator->odom_prediction_error()) {
            status.prediction_labels.push_back(std_msgs::msg::String());
            status.prediction_labels.back().data = "odom";
            status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->odom_prediction_error().get().cast<double>())).transform);
        }

        status_pub->publish(status);
    }

    void TimerCallback()
    {
        double total_ms = 0.0, average_ms = 0.0;
        double average_fps = 0;
        {
            std::lock_guard<std::mutex> lock(timer_data_mutex_);
            for(int i=0; i<ms_deque_.size(); i++){
                total_ms += ms_deque_.at(i);
            }
            average_ms = ms_deque_.empty() ? 0.0 : total_ms / ms_deque_.size();
            average_fps = static_cast<double>(fps_count_)/10;
            ms_deque_.clear();
            fps_count_ = 0;
        }
        RCLCPP_INFO(get_logger(), "average pointcloud process time in %d s is %lf ms, fps is: %lf", print_ms_per_sec, average_ms, average_fps);

    }
    rclcpp::TimerBase::SharedPtr timer_;

private:
    bool enable_robot_odometry_prediction;
    double max_correspondence_dist;
    double max_valid_point_dist;        
    double ndt_neighbor_search_radius;
    double ndt_resolution;
    double manual_pose_translation_gate;
    double manual_pose_yaw_gate;
    bool auto_reanchor_enabled;
    int auto_reanchor_samples;
    double auto_reanchor_position_span;
    double auto_reanchor_yaw_span;
    double auto_reanchor_max_translation;
    double auto_reanchor_max_yaw;
    double auto_reanchor_max_fitness;
    int manual_gate_violation_samples;
    double ndt_fusion_gain;
    double ndt_fusion_max_translation_step;
    double ndt_fusion_max_yaw_step;
    std::string ndt_neighbor_search_method;
    std::string reg_method;
    std::string robot_odom_frame_id;
    std::string robot_odom_child_frame_id;
    std::string odom_child_frame_id;
    bool use_imu;
    bool invert_acc;
    bool invert_gyro;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leg_odom2_sub;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub;
    rclcpp::Publisher<msg::ScanMatchingStatus>::SharedPtr status_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reanchor_pub;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    nav_msgs::msg::Odometry::SharedPtr leg_odom2_msg;
    std::mutex leg_odom_mutex_;
    std::mutex manual_anchor_mutex_;
    Eigen::Matrix4f manual_map_anchor_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f leg_odom_anchor_ = Eigen::Matrix4f::Identity();
    bool manual_anchor_valid_ = false;
    bool correction_rejected_ = false;
    bool auto_reanchor_pending_ = false;
    int consecutive_gate_violations_ = 0;
    std::deque<std::array<double, 3>> auto_reanchor_candidates_;

    // imu input buffer
    std::mutex imu_data_mutex;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_data_buffer_;

    // globalmap and registration method
    pcl::PointCloud<PointT>::Ptr globalmap;
    pcl::VoxelGrid<PointT> downsample_filter_;
    pcl::Registration<PointT, PointT>::Ptr registration;

    // pose estimator
    std::mutex pose_estimator_mutex;
    std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;

    // global localization
    bool use_global_localization;
    std::atomic_bool relocalizing;
    std::unique_ptr<DeltaEstimater> delta_estimater;

    pcl::PointCloud<PointT>::ConstPtr last_scan;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr relocalize_server;
    rclcpp::Client<hdl_global_localization::srv::SetGlobalMap>::SharedPtr set_global_map_service;
    rclcpp::Client<hdl_global_localization::srv::QueryGlobalLocalization>::SharedPtr query_global_localization_service;

    double t_diff_;
    rclcpp::Time last_lidar_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
    rclcpp::Time last_imu_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
    std::mutex timer_data_mutex_;
    std::deque<double> ms_deque_;
    int fps_count_ = 0;
};  //class

}   //namespace

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hdl_localization::HdlLocalizationApp)
