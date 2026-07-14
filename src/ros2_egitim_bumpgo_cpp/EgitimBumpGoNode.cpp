#include "ros2_egitim_bumpgo_cpp/EgitimBumpGoNode.hpp"
#include <cmath>
#include <limits>

namespace ros2_egitim_bumpgo_cpp 
{
    using std::placeholders::_1;

    EgitimBumpGoNode::EgitimBumpGoNode() : Node("egitim_bumpgo")
    {
        declare_parameter<double>("obstacle_distance", 0.5);
        declare_parameter<double>("front_angle", 45.0);
        declare_parameter<double>("linear_speed", 0.2);
        declare_parameter<double>("angular_speed", 0.3);
        declare_parameter<double>("backup_speed", 0.1);
        declare_parameter<double>("backing_time", 2.0);
        declare_parameter<double>("turning_time", 2.0);

        obstacle_distance_ = get_parameter("obstacle_distance").as_double();
        front_angle_ = get_parameter("front_angle").as_double();
        linear_speed_ = get_parameter("linear_speed").as_double();
        angular_speed_ = get_parameter("angular_speed").as_double();
        backup_speed_ = get_parameter("backup_speed").as_double();
        backing_time_ = get_parameter("backing_time").as_double();
        turning_time_ = get_parameter("turning_time").as_double();


        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&EgitimBumpGoNode::scan_callback, this, _1)
        );

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(get_logger(), "Egitim BumpGo hazır!");

        state_ = RobotState::DRIVING;

        state_ts_ = now();

    }

    void EgitimBumpGoNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        const float min_front = min_range_in_front(*msg);

        if(!std::isfinite(min_front))
        {
            RCLCPP_WARN(get_logger(), "Ön bölgede geçerli ölçüm yok");
            return;
        }

        geometry_msgs::msg::Twist output_vel;

        switch (state_) 
        {
            case RobotState::DRIVING:
                RCLCPP_WARN(get_logger(), "New State: DRIVING");
                if(min_front < obstacle_distance_)
                {
                    change_state(RobotState::BACKING);
                    output_vel.linear.x = -backup_speed_;
                    break;
                }

                output_vel.linear.x = linear_speed_;
                break;

            case RobotState::BACKING:
                RCLCPP_WARN(get_logger(), "New State: BACKING");
                if((now() - state_ts_).seconds() >= backing_time_)
                {
                    change_state(RobotState::TURNING);
                    output_vel.angular.z = angular_speed_;
                    break;
                }
                output_vel.linear.x = -backup_speed_;
                break;

            case RobotState::TURNING:
                RCLCPP_WARN(get_logger(), "New State: TURNING");
                if((now() - state_ts_).seconds() >= turning_time_)
                {
                    change_state(RobotState::DRIVING);
                    output_vel.linear.x = linear_speed_;
                    break;
                }
                output_vel.angular.z = angular_speed_;
                break;

        }

        cmd_vel_pub_->publish(output_vel);

    }

    float EgitimBumpGoNode::min_range_in_front(const sensor_msgs::msg::LaserScan & scan) const 
    {
        float min_dist  = std::numeric_limits<float>::infinity();

        const float front_angle_limit = front_angle_ * M_PI / 180;

        float angle = scan.angle_min;

        for(size_t i=0; i < scan.ranges.size(); i++)
        {
            const float range = scan.ranges[i];

            if(std::abs(angle) <= front_angle_limit &&
                std::isfinite(range) &&
                range >= scan.range_min &&
                range <= scan.range_max &&
                range < min_dist)
            {
                min_dist = range;
            }

            angle += scan.angle_increment;
        }

        return min_dist;
    }

    void EgitimBumpGoNode::change_state(RobotState new_state)
    {
        state_ = new_state;
        state_ts_ = now();
    }
}