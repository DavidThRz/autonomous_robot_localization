
#include "rclcpp/rclcpp.hpp"


int main(int argc, char **argv) 
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("nodo_loop_manual");
  rclcpp::Rate rate(10);

  while (rclcpp::ok()) 
  {
    RCLCPP_INFO(node->get_logger(), "Iteración del loop");

    // Equivale a spinOnce
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
}