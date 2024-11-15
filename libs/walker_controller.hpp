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
#include <iostream>

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
        /**
         * @brief Construct a new Moving Forward State object
         * 
         */
        MovingForwardState();

        /**
         * @brief Function to handle the action for the current state
         * 
         * @return geometry_msgs::msg::Twist 
         */
        geometry_msgs::msg::Twist handle() override;

        /**
         * @brief Function to update the state based on sensor input
         * 
         * @param close_to_obstacle 
         */
        void update_state(bool close_to_obstacle) override;
};

/**
* @brief Class representing the "Rotating" state of the robot
*/
class RotatingState : public State {
    public:
        /**
         * @brief Construct a new Rotating State object
         * 
         * @param clockwise 
         */
        explicit RotatingState(bool clockwise);

        /**
         * @brief Function to handle the action for the current state
         * 
         * @return geometry_msgs::msg::Twist 
         */
        geometry_msgs::msg::Twist handle() override;

        /**
         * @brief Function to update the state based on sensor input
         * 
         * @param close_to_obstacle 
         */
        void update_state(bool close_to_obstacle) override;

    private:
        bool clockwise_;
};

/**
* @brief Context class for the robot that manages state transitions
*/
class WalkerController {
    public:
        /**
         * @brief Construct a new Walker Controller object
         * 
         */
        WalkerController();

        /**
         * @brief Destroy the Walker Controller object
         * 
         */
        ~WalkerController();

        /**
         * @brief Function to update the state based on sensor input
         * 
         * @param distance_to_obstacle 
         * @return geometry_msgs::msg::Twist 
         */
        geometry_msgs::msg::Twist update(double distance_to_obstacle);

    private:

        /**
         * @brief State pointer to the current state of the robot
         * 
         */
        State* current_state;

        /**
         * @brief Boolean indicating if the robot is close to an obstacle
         * 
         */
        bool close_to_obstacle;

        /**
         * @brief Boolean indicating if the robot should rotate clockwise
         * 
         */
        bool rotate_clockwise;

        /**
         * @brief Set the state object
         * 
         * @param state 
         */
        void set_state(State* state);

        /**
         * @brief Boolean indicating if the robot is currently turning
         * 
         */
        bool turning;
};
