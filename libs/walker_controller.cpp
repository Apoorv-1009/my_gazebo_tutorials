/**
 * @file walker_controller.cpp
 * @author Apoorv Thapliyal
 * @brief C++ source file for walker_controller class
 * @version 0.1
 * @date 2024-11-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "walker_controller.hpp"

/**
* @brief Construct a new Moving Forward State:: Moving Forward State object
* 
*/
MovingForwardState::MovingForwardState() {}

/**
* @brief Function to handle the action for the current state
* 
* @return geometry_msgs::msg::Twist 
*/
geometry_msgs::msg::Twist MovingForwardState::handle() {
    geometry_msgs::msg::Twist action;
    // Move forward
    action.linear.x = 0.3;  
    // No rotation
    action.angular.z = 0.0; 
    return action;
}

/**
* @brief Function to update the state based on sensor input
* 
* @param close_to_obstacle 
*/
void MovingForwardState::update_state(bool close_to_obstacle) {
    // No internal state change; transitions are handled in the context class
    // std::cout << "Moving Forward \n";
}

/**
* @brief Construct a new Rotating State:: Rotating State object
* 
* @param clockwise Boolean indicating the direction of rotation
*/
RotatingState::RotatingState(bool clockwise) : clockwise_(clockwise) {}

geometry_msgs::msg::Twist RotatingState::handle() {
    geometry_msgs::msg::Twist action;
    action.linear.x = 0.0;                   // No forward motion
    action.angular.z = clockwise_ ? 0.5 : -0.5; // Rotate clockwise or counterclockwise
    return action;
}

/**
* @brief Function to update the state based on sensor input
* 
* @param close_to_obstacle 
*/
void RotatingState::update_state(bool close_to_obstacle) {
    // No internal state change; transitions are handled in the context class
    // std::cout << "Rotating \n";
}

/**
 * @brief Construct a new Walker Controller:: Walker Controller object
 * 
 */
WalkerController::WalkerController()
    : current_state(new MovingForwardState()), close_to_obstacle(false), rotate_clockwise(true), turning(false) {}

/**
* @brief Destroy the Walker Controller:: Walker Controller object
* 
*/
WalkerController::~WalkerController() {
    delete current_state;
}

/**
* @brief Function to set the current state of the robot
* 
* @param state 
*/
void WalkerController::set_state(State* state) {
    delete current_state;
    current_state = state;
}

/**
    * @brief Function to update the robot state based on distance to obstacle
    * 
    * @param distance_to_obstacle 
    * @return geometry_msgs::msg::Twist 
    */
geometry_msgs::msg::Twist WalkerController::update(double distance_to_obstacle) {
    // Determine if the robot is close to an obstacle
    close_to_obstacle = (distance_to_obstacle < 1.0);

    // Update the current state based on sensor data
    current_state->update_state(close_to_obstacle);

    // Handle the current state's action
    geometry_msgs::msg::Twist action = current_state->handle();

    // std::cout << "Close to obstacle: " << close_to_obstacle << "\n";

    // Transition to RotatingState if close to obstacle
    if (close_to_obstacle) {

        if (turning) {
            return action;
        }
        
        // Alternate rotation direction
        rotate_clockwise = !rotate_clockwise; 
        
        // Set turning flag to true
        turning = true;
        
        // std::cout << "Rotating clockwise: " << rotate_clockwise << "\n";
        // Set the state to RotatingState
        set_state(new RotatingState(rotate_clockwise));
    } 

    // Transition to MovingForwardState if path is clear
    else {
        
        // Reset turning flag
        turning = false;

        // Set the state to MovingForwardState
        set_state(new MovingForwardState());
    }

    return action;
}
