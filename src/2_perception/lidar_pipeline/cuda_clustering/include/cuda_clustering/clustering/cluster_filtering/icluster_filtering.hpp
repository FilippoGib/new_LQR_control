#pragma once
#include <geometry_msgs/msg/point.hpp>
#include <optional>

class IClusterFiltering
{
    public:
        virtual std::optional<geometry_msgs::msg::Point> analiseCluster(float* cluster, unsigned int points_num) = 0;
};