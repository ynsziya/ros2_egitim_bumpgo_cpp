#ifndef ROS2_EGITIM_BUMPGO_CPP__EGITIMBUMPGONODE_HPP_
#define ROS2_EGITIM_BUMPGO_CPP__EGITIMBUMPGONODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace ros2_egitim_bumpgo_cpp 
{
    enum class RobotState
    {
        DRIVING,
        BACKING,
        TURNING
    };


    class EgitimBumpGoNode : public rclcpp::Node
    {
        public:
            EgitimBumpGoNode();

        private:
            void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
            float min_range_in_front(const sensor_msgs::msg::LaserScan & scan) const;
            void change_state(RobotState new_state);

            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

            RobotState state_{RobotState::DRIVING};

            double obstacle_distance_{0.5};
            double front_angle_{30.0};
            double linear_speed_{0.2};
            double angular_speed_{0.3};
            double backup_speed_{0.3};

            double backing_time_{2.0};
            double turning_time_{2.0};

            rclcpp::Time state_ts_;

    };
}


#endif