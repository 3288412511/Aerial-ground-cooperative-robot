#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"

using namespace std::chrono_literals;

struct Waypoint
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double speed{0.3};
    double yaw{0.0};
};

class MavrosCsvAreaMission : public rclcpp::Node
{
public:
    MavrosCsvAreaMission() : Node("mavros_csv_area_mission")
    {
        waypoint_csv_ = this->declare_parameter<std::string>("waypoint_csv", "");
        reach_threshold_ = this->declare_parameter<double>("reach_threshold", 0.25);
        default_speed_ = this->declare_parameter<double>("default_speed", 0.3);
        auto_start_after_offboard_ = this->declare_parameter<bool>("auto_start_after_offboard", true);

        if (!load_waypoints(waypoint_csv_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoint CSV. waypoint_csv='%s'", waypoint_csv_.c_str());
        }

        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state",
            10,
            std::bind(&MavrosCsvAreaMission::state_callback, this, std::placeholders::_1)
        );

        local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose",
            10,
            std::bind(&MavrosCsvAreaMission::local_pose_callback, this, std::placeholders::_1)
        );

        setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local",
            10
        );

        timer_ = this->create_wall_timer(50ms, std::bind(&MavrosCsvAreaMission::timer_callback, this));
        last_time_ = this->now();

        RCLCPP_WARN(this->get_logger(), "CSV area mission node started. Loaded %zu waypoints.", waypoints_.size());
    }

private:
    bool load_waypoints(const std::string &path)
    {
        waypoints_.clear();
        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter waypoint_csv is empty.");
            return false;
        }

        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open waypoint CSV: %s", path.c_str());
            return false;
        }

        std::string line;
        bool first_line = true;
        while (std::getline(file, line)) {
            if (line.empty()) {
                continue;
            }
            if (first_line) {
                first_line = false;
                if (line.find("index") != std::string::npos || line.find("x") != std::string::npos) {
                    continue;
                }
            }

            std::vector<std::string> cols;
            std::stringstream ss(line);
            std::string item;
            while (std::getline(ss, item, ',')) {
                cols.push_back(item);
            }

            // Expected: index,x,y,z,speed,yaw
            // Also accept: x,y,z or x,y,z,speed,yaw
            try {
                Waypoint wp;
                if (cols.size() >= 6) {
                    wp.x = std::stod(cols[1]);
                    wp.y = std::stod(cols[2]);
                    wp.z = std::stod(cols[3]);
                    wp.speed = std::stod(cols[4]);
                    wp.yaw = std::stod(cols[5]);
                } else if (cols.size() == 5) {
                    wp.x = std::stod(cols[0]);
                    wp.y = std::stod(cols[1]);
                    wp.z = std::stod(cols[2]);
                    wp.speed = std::stod(cols[3]);
                    wp.yaw = std::stod(cols[4]);
                } else if (cols.size() == 3) {
                    wp.x = std::stod(cols[0]);
                    wp.y = std::stod(cols[1]);
                    wp.z = std::stod(cols[2]);
                    wp.speed = default_speed_;
                    wp.yaw = 0.0;
                } else {
                    continue;
                }

                if (wp.speed <= 0.0) {
                    wp.speed = default_speed_;
                }
                waypoints_.push_back(wp);
            } catch (const std::exception &e) {
                RCLCPP_WARN(this->get_logger(), "Skip invalid CSV line: %s", line.c_str());
            }
        }

        return !waypoints_.empty();
    }

    void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        is_connected_ = msg->connected;
        is_armed_ = msg->armed;
        current_mode_ = msg->mode;
        bool now_offboard = (msg->mode == "OFFBOARD");

        if (now_offboard != is_offboard_) {
            if (now_offboard) {
                RCLCPP_WARN(this->get_logger(), "PX4 entered OFFBOARD mode.");
                if (auto_start_after_offboard_) {
                    mission_started_ = true;
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "PX4 left OFFBOARD mode. Current mode: %s", msg->mode.c_str());
            }
        }
        is_offboard_ = now_offboard;
    }

    void local_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        has_local_pose_ = true;
    }

    void timer_callback()
    {
        if (waypoints_.empty()) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "No waypoints loaded.");
            return;
        }

        // Before OFFBOARD, keep sending the first setpoint so PX4 can enter OFFBOARD.
        if (!is_connected_) {
            publish_pose(waypoints_.front());
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for MAVROS connection...");
            return;
        }

        if (!is_offboard_) {
            publish_pose(waypoints_.front());
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Waiting for OFFBOARD. Current mode: %s", current_mode_.c_str()
            );
            return;
        }

        if (!has_local_pose_) {
            publish_pose(waypoints_.front());
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for local pose...");
            return;
        }

        if (!mission_started_) {
            publish_pose(waypoints_.front());
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "OFFBOARD active, mission not started.");
            return;
        }

        if (mission_finished_) {
            publish_pose(waypoints_.back());
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Mission finished. Holding last waypoint.");
            return;
        }

        update_mission();
    }

    void update_mission()
    {
        if (current_index_ >= waypoints_.size()) {
            mission_finished_ = true;
            publish_pose(waypoints_.back());
            return;
        }

        Waypoint target = waypoints_[current_index_];
        double d = distance_to(target);
        if (d < reach_threshold_) {
            RCLCPP_WARN(
                this->get_logger(),
                "Reached waypoint %zu/%zu: x=%.2f y=%.2f z=%.2f",
                current_index_ + 1, waypoints_.size(), target.x, target.y, target.z
            );
            current_index_++;
            virtual_initialized_ = false;
            if (current_index_ >= waypoints_.size()) {
                mission_finished_ = true;
                publish_pose(waypoints_.back());
                RCLCPP_WARN(this->get_logger(), "All waypoints reached.");
                return;
            }
            target = waypoints_[current_index_];
        }

        Waypoint smooth = get_smooth_setpoint(target);
        publish_pose(smooth);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Flying waypoint %zu/%zu | target=(%.2f, %.2f, %.2f) | distance=%.2f | speed=%.2f",
            current_index_ + 1, waypoints_.size(), target.x, target.y, target.z, d, target.speed
        );
    }

    Waypoint get_smooth_setpoint(const Waypoint &target)
    {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0 || dt > 1.0) {
            dt = 0.05;
        }

        if (!virtual_initialized_) {
            virtual_wp_.x = current_pose_.pose.position.x;
            virtual_wp_.y = current_pose_.pose.position.y;
            virtual_wp_.z = current_pose_.pose.position.z;
            virtual_wp_.speed = target.speed;
            virtual_wp_.yaw = target.yaw;
            virtual_initialized_ = true;
        }

        double dx = target.x - virtual_wp_.x;
        double dy = target.y - virtual_wp_.y;
        double dz = target.z - virtual_wp_.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < 1e-6) {
            return target;
        }

        double step = target.speed * dt;
        if (step >= dist) {
            virtual_wp_ = target;
            return virtual_wp_;
        }

        virtual_wp_.x += dx / dist * step;
        virtual_wp_.y += dy / dist * step;
        virtual_wp_.z += dz / dist * step;
        virtual_wp_.speed = target.speed;
        virtual_wp_.yaw = target.yaw;
        return virtual_wp_;
    }

    double distance_to(const Waypoint &wp) const
    {
        double dx = current_pose_.pose.position.x - wp.x;
        double dy = current_pose_.pose.position.y - wp.y;
        double dz = current_pose_.pose.position.z - wp.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    void publish_pose(const Waypoint &wp)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = wp.x;
        pose.pose.position.y = wp.y;
        pose.pose.position.z = wp.z;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(wp.yaw / 2.0);
        pose.pose.orientation.w = std::cos(wp.yaw / 2.0);
        setpoint_pub_->publish(pose);
    }

private:
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string waypoint_csv_;
    std::string current_mode_{"UNKNOWN"};

    std::vector<Waypoint> waypoints_;
    geometry_msgs::msg::PoseStamped current_pose_;
    Waypoint virtual_wp_;

    bool is_connected_{false};
    bool is_armed_{false};
    bool is_offboard_{false};
    bool has_local_pose_{false};
    bool mission_started_{false};
    bool mission_finished_{false};
    bool virtual_initialized_{false};
    bool auto_start_after_offboard_{true};

    size_t current_index_{0};
    double reach_threshold_{0.25};
    double default_speed_{0.3};
    rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MavrosCsvAreaMission>());
    rclcpp::shutdown();
    return 0;
}
