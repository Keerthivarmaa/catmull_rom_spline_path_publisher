#include "rclcpp/rclcpp.hpp"
#include "spline_path_generator/SplineNode.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SplineNode>());
  rclcpp::shutdown();
  return 0;
}

