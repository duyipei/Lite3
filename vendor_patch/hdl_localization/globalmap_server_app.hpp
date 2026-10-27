
#pragma once

#include <mutex>
#include <memory>
#include <iostream>
#include <cmath>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <pcl_ros/point_cloud.hpp>
#include <pcl_ros/transforms.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/voxel_grid.h>

namespace hdl_localization {

class GlobalmapServerApp : public rclcpp::Node{
public:
    using PointT = pcl::PointXYZI;
    GlobalmapServerApp(rclcpp::NodeOptions options) : Node("global_map_server", options){
        initialize_params();

        // publish globalmap with "latched" publisher
        globalmap_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/globalmap", 5);
        map_update_sub = create_subscription<std_msgs::msg::String>(
            "/map_request/pcd", 
            10, 
            [this](const std_msgs::msg::String::SharedPtr msg){
                this->map_update_callback(msg);
            }
        );
        using namespace std::chrono_literals;
        globalmap_pub_timer = create_wall_timer(1000ms, std::bind(&GlobalmapServerApp::pub_once_cb, this));
    }
    virtual ~GlobalmapServerApp(){
    }

private:
    void initialize_params() {
        // read globalmap from a pcd file
        std::string globalmap_pcd;
        declare_parameter("globalmap_pcd", "");
        if(get_parameter("globalmap_pcd", globalmap_pcd)){
            RCLCPP_INFO(get_logger(), "globalmap_pcd is: %s", globalmap_pcd.c_str());
        }
        globalmap.reset(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
        globalmap->header.frame_id = "map";

        std::ifstream utm_file(globalmap_pcd + ".utm");
        bool convert_utm_to_local;
        declare_parameter("convert_utm_to_local", true);
        if(get_parameter("convert_utm_to_local", convert_utm_to_local)){
            RCLCPP_INFO(get_logger(), "convert_utm_to_local is: %d", convert_utm_to_local);
        }        
        if (utm_file.is_open() && convert_utm_to_local) {
        double utm_easting;
        double utm_northing;
        double altitude;
        utm_file >> utm_easting >> utm_northing >> altitude;
        for(auto& pt : globalmap->points){
            pt.getVector3fMap() -= Eigen::Vector3f(utm_easting, utm_northing, altitude);
        }
        RCLCPP_INFO_STREAM(get_logger(), "Global map offset by UTM reference coordinates (x = "
            << utm_easting << ", y = " << utm_northing << ") and altitude (z = " << altitude << ")");
        }

        // downsample globalmap
        double downsample_resolution;
        declare_parameter("downsample_resolution", 0.1);
        if(get_parameter("downsample_resolution", downsample_resolution)){
            RCLCPP_INFO(get_logger(), "downsample_resolution is: %lf", downsample_resolution);
        }
        // Both composed nodes receive the same launch parameters. The source
        // scan's 0.3/0.5-m CPU filter must not silently erase the reference
        // map's finer geometry. Preserve legacy behavior unless overridden.
        declare_parameter("globalmap_downsample_resolution", downsample_resolution);
        get_parameter("globalmap_downsample_resolution", downsample_resolution);
        if(!std::isfinite(downsample_resolution) || downsample_resolution <= 0.0){
            throw std::invalid_argument("globalmap_downsample_resolution must be positive and finite");
        }
        RCLCPP_INFO(get_logger(), "globalmap_downsample_resolution is: %lf", downsample_resolution);
        boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
        voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
        voxelgrid->setInputCloud(globalmap);

        pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
        voxelgrid->filter(*filtered);

        RCLCPP_INFO(get_logger(), "reference map points: %zu -> %zu", globalmap->size(), filtered->size());

        globalmap = filtered;
    }

    void pub_once_cb(){
        sensor_msgs::msg::PointCloud2 ros2_pcl;
        pcl::toROSMsg(*globalmap, ros2_pcl);
        ros2_pcl.header.frame_id = "map";
        globalmap_pub->publish(ros2_pcl);
    }

    void map_update_callback(const std_msgs::msg::String::SharedPtr msg){
        RCLCPP_INFO_STREAM(get_logger(), "Received map request, map path : " << msg->data);
        std::string globalmap_pcd = msg->data;
        globalmap.reset(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
        globalmap->header.frame_id = "map";

        // downsample globalmap
        double downsample_resolution;
        declare_parameter("downsample_resolution", 0.1);
        if(get_parameter("downsample_resolution", downsample_resolution)){
            RCLCPP_INFO(get_logger(), "downsample_resolution is: %lf", downsample_resolution);
        }
        boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
        voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
        voxelgrid->setInputCloud(globalmap);

        pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
        voxelgrid->filter(*filtered);

        globalmap = filtered;
        sensor_msgs::msg::PointCloud2 ros2_pcl;
        pcl::toROSMsg(*globalmap, ros2_pcl);
        ros2_pcl.header.frame_id = "map";
        globalmap_pub->publish(ros2_pcl);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_update_sub;

    rclcpp::TimerBase::SharedPtr globalmap_pub_timer;
    pcl::PointCloud<PointT>::Ptr globalmap;
};

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hdl_localization::GlobalmapServerApp)
