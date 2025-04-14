#pragma once 
#include <string.h>

#include "cuda_clustering/clustering/cuda_clustering.hpp"
#include "cuda_clustering/filtering/cuda_filtering.hpp"
#include "cuda_clustering/clustering/iclustering.hpp"
#include "cuda_clustering/filtering/ifiltering.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


class ControllerNode : public rclcpp::Node
{
    private:
        std::shared_ptr<visualization_msgs::msg::Marker> cones{new visualization_msgs::msg::Marker()};
        std::string input_topic, frame_id;
        float voxelX, voxelY, voxelZ, clusterMaxX, clusterMaxY, clusterMaxZ, maxHeight;
        unsigned int countThreshold, minClusterSize, maxClusterSize;
        bool filterOnZ;

        IFilter *filter;
        IClustering *clustering;

        /* Publisher */
        //rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;

        /* Subscriber */
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cones_array_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cp_pub;

        /* Load parameters function */
        void loadParameters();

        /* PointCloud Callback */
        void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr sub_cloud);
    public:
        ControllerNode();
};