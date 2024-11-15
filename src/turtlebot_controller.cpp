/**
 * @file turtlebot_controller.cpp
 * @author Apoorv Thapliyal
 * @brief C++ source file for the turtlebot_controller node
 * @version 0.1
 * @date 2024-11-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TurtlebotController : public rclcpp::Node {
    private:

        /**
         * @brief Subscriber to the laser scan topic
         * 
         */
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber;

        /**
         * @brief Publisher for the velocity command topic
         * 
         */
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;

        /**
        * @brief Timer to publish velocity commands periodically
        * 
        */
        rclcpp::TimerBase::SharedPtr timer;

    public:

        TurtlebotController() : Node("turtlebot_controller") {

            // Initialize the /scan subscriber 
            laser_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 10, std::bind(&TurtlebotController::laser_callback, this, std::placeholders::_1));

            // Initialize the /cmd_vel publisher
            velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            // Initialize the timer to publish velocity commands periodically
            timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TurtlebotController::publish_velocity, this));
        }

        /**
         * @brief Callback function for the laser scan topic
         * 
         * @param msg 
         */
        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            // Print the distance of the first ray
            RCLCPP_INFO(this->get_logger(), "Distance to obstacle: %f", msg->ranges[0]);
        }

        /**
         * @brief Callback function to publish velocity commands
         * 
         */
        void publish_velocity() {
            // Create a velocity command message
            auto velocity_msg = geometry_msgs::msg::Twist();
            velocity_msg.linear.x = 0.0;  // Move forward
            velocity_msg.angular.z = 0.0; // No rotation

            // Publish the velocity command
            velocity_publisher->publish(velocity_msg);
        }
};

/**
 * @brief main function to create and run the turtlebot_controller node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtlebotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

