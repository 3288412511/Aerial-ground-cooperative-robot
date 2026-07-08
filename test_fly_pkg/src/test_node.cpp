#include "rclcpp/rclcpp.hpp"

int main(int argc , char **argv)
{
    rclcpp::init(argc , argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test_node");
    RCLCPP_INFO(node ->get_logger() , "Hello,test_node!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}