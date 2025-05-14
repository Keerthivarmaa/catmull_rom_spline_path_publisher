#ifndef SPLINENODE_H
#define SPLINENODE_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include "spline_path_generator/Spline.h"

class SplineNode : public rclcpp::Node {
public:
    // Constructor
    SplineNode();

private:
    // Function to publish the spline path
    void publishSpline();

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // SPLINENODE_H

