/**
 * @file walker_controller.hpp
 * @author Apoorv Thapliyal
 * @brief C++ header file for walker_controller class
 * @version 0.1
 * @date 2024-11-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include "geometry_msgs/msg/twist.hpp"

/**
 * @brief Namespace for the walker controller
 */
namespace walker {

    /**
     * @brief Abstract base class for robot states
     */
    class State {
    public:
        virtual ~State() = default;

        /**
         * @brief Handles the action for the current state
         * @return geometry_msgs::msg::Twist The action to be taken (velocity command)
         */
        virtual geometry_msgs::msg::Twist handle() = 0;

        /**
         * @brief Updates the state based on sensor input
         * @param close_to_obstacle Boolean indicating if the robot is close to an obstacle
         */
        virtual void update_state(bool close_to_obstacle) = 0;
    };

    /**
     * @brief Class representing the "Moving Forward" state of the robot
     */
    class MovingForwardState : public State {
    public:
        MovingForwardState();
        geometry_msgs::msg::Twist handle() override;
        void update_state(bool close_to_obstacle) override;
    };

    /**
     * @brief Class representing the "Rotating" state of the robot
     */
    class RotatingState : public State {
    public:
        explicit RotatingState(bool clockwise);
        geometry_msgs::msg::Twist handle() override;
        void update_state(bool close_to_obstacle) override;

    private:
        bool clockwise_;
    };

    /**
     * @brief Context class for the robot that manages state transitions
     */
    class WalkerController {
    public:
        WalkerController();
        ~WalkerController();
        geometry_msgs::msg::Twist update(double distance_to_obstacle);

    private:
        State* current_state;
        bool close_to_obstacle;
        bool rotate_clockwise;
        void set_state(State* state);
    };
}
