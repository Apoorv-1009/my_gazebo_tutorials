/**
 * @file turtlebot_controller.cpp
 * @author Apoorv Thapliyal
 * @brief C++ source file for the turtlebot_controller node
 * @version 0.1
 * @date 2024-11-15
 *
 * @copyright Copyright (c) 2024
 * @license MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <algorithm>
#include <geometry_msgs/msg/twist.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "walker_controller.hpp"
#include <memory>

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

  /**
   * @brief Double to store the distance to the closest obstacle
   *
   */
  double closest_obstacle_distance;

  /**
   * @brief Instance of the walker controller class
   *
   */
  WalkerController walker_controller;

 public:
  TurtlebotController() : Node("turtlebot_controller") {
    // Initialize the /scan subscriber
    laser_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&TurtlebotController::laser_callback, this,
                  std::placeholders::_1));

    // Initialize the /cmd_vel publisher
    velocity_publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Initialize the timer to publish velocity commands periodically
    timer = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&TurtlebotController::publish_velocity, this));
  }

  /**
   * @brief Callback function for the laser scan topic
   *
   * @param msg
   */
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Print the distance of the first ray
    // RCLCPP_INFO(this->get_logger(), "Distance to obstacle: %f",
    // msg->ranges[0]);

    // Update the distance to the closest obstacle, get the minimum value from
    // the first 30 rays on either side
    closest_obstacle_distance = std::min(
        *std::min_element(msg->ranges.begin(), msg->ranges.begin() + 10),
        *std::min_element(msg->ranges.rbegin(), msg->ranges.rbegin() + 10));
  }

  /**
   * @brief Callback function to publish velocity commands
   *
   */
  void publish_velocity() {
    RCLCPP_INFO(this->get_logger(), "Closest obstacle distance: %f",
                closest_obstacle_distance);

    geometry_msgs::msg::Twist velocity_msg =
        walker_controller.update(closest_obstacle_distance);

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
