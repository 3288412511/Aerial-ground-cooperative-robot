#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"

using namespace std::chrono_literals;

class MavrosOffboardTarget : public rclcpp::Node
{
public:
    MavrosOffboardTarget() : Node("mavros_offboard_target")
    {
        // MAVROS 使用 ROS 常见的 ENU 坐标系：
        // x: 前方/东向，y: 左方/北向，z: 向上
        // 注意：这里不是 PX4 内部 NED 坐标系
        target_x_ = this->declare_parameter<double>("target_x", 2.0);
        target_y_ = this->declare_parameter<double>("target_y", 0.0);
        target_z_ = this->declare_parameter<double>("target_z", 10.0);
        target_yaw_ = this->declare_parameter<double>("target_yaw", 0.0);

        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state",
            10,
            std::bind(&MavrosOffboardTarget::state_callback, this, std::placeholders::_1)
        );

        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local",
            10
        );

        timer_ = this->create_wall_timer(
            50ms,
            std::bind(&MavrosOffboardTarget::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "MAVROS Offboard target node started.");
        RCLCPP_INFO(
            this->get_logger(),
            "Target position ENU: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f rad",
            target_x_, target_y_, target_z_, target_yaw_
        );
    }

private:
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_mode_ = msg->mode;
        is_armed_ = msg->armed;
        is_connected_ = msg->connected;

        bool now_offboard = (msg->mode == "OFFBOARD");

        if (now_offboard != is_offboard_) {
            if (now_offboard) {
                RCLCPP_WARN(this->get_logger(), "PX4 has entered OFFBOARD mode.");
            } else {
                RCLCPP_WARN(
                    this->get_logger(),
                    "PX4 is not in OFFBOARD mode. Current mode: %s",
                    msg->mode.c_str()
                );
            }
        }

        is_offboard_ = now_offboard;
    }

    void timer_callback()
    {
        if (!is_connected_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for MAVROS connection..."
            );
            return;
        }

        if (!is_offboard_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Current mode is %s. Waiting for OFFBOARD mode...",
                current_mode_.c_str()
            );

            // 关键点：
            // PX4 切入 Offboard 前通常要求已经在持续接收 setpoint。
            // 所以这里即使还没进入 OFFBOARD，也先发布目标点作为 setpoint 流。
            // 这样你用遥控器或 QGC 切 OFFBOARD 时才更容易成功。
            publish_target_pose();
            return;
        }

        // 已经进入 OFFBOARD，持续发布目标点
        publish_target_pose();

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "OFFBOARD active. Publishing target: x=%.2f, y=%.2f, z=%.2f",
            target_x_, target_y_, target_z_
        );
    }

    void publish_target_pose()
    {
        geometry_msgs::msg::PoseStamped pose;

        pose.header.stamp = this->now();
        pose.header.frame_id = "map";

        pose.pose.position.x = target_x_;
        pose.pose.position.y = target_y_;
        pose.pose.position.z = target_z_;

        // 只设置 yaw，roll 和 pitch 保持 0
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(target_yaw_ / 2.0);
        pose.pose.orientation.w = std::cos(target_yaw_ / 2.0);

        local_pos_pub_->publish(pose);
    }

private:
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool is_connected_{false};
    bool is_armed_{false};
    bool is_offboard_{false};

    std::string current_mode_{"UNKNOWN"};

    double target_x_{2.0};
    double target_y_{0.0};
    double target_z_{1.5};
    double target_yaw_{0.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MavrosOffboardTarget>());
    rclcpp::shutdown();
    return 0;
}