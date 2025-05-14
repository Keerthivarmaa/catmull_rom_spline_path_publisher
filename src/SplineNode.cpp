#include "spline_path_generator/SplineNode.h"
#include "spline_path_generator/Spline.h"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

SplineNode::SplineNode() : Node("spline_node") {
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("spline_path", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     std::bind(&SplineNode::publishSpline, this));
}

void SplineNode::publishSpline() {
    std::vector<Point2D> control_points = {
        {0.0, 0.0},  // P0
        {1.0, 2.0},  // P1
        {4.0, 2.0},  // P2
        {5.0, 0.0}  // P3
        
    };

    Spline spline(control_points);
    std::vector<Point2D> spline_points = spline.computeCentripetalCatmullRomSpline();

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "map";

    for (const auto& pt : spline_points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;  // Identity orientation
        path_msg.poses.push_back(pose);
    }

    path_publisher_->publish(path_msg);
    /*RCLCPP_INFO(this->get_logger(), "Published Centripetal Catmull-Rom spline path with %zu points.", spline_points.size());
     // Log the control points
    RCLCPP_INFO(this->get_logger(), "Control Points:");
    for (const auto& pt : control_points) {
      RCLCPP_INFO(this->get_logger(), "(%.2f, %.2f)", pt.x, pt.y);
    }*/

    // Log the start and end points
    // if (!spline_points.empty()) {
    //   RCLCPP_INFO(this->get_logger(), "Start Point: (%.2f, %.2f)", spline_points.front().x, spline_points.front().y);
    //   RCLCPP_INFO(this->get_logger(), "End Point:   (%.2f, %.2f)", spline_points.back().x, spline_points.back().y);
    // }
  
}

