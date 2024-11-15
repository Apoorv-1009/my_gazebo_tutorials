[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

# ROS 2 Working with Gazebo Classic - ENPM700

This repository features the Gazebo Classic Tutorials package developed as part of the ENPM700 course. This project was created by **Apoorv Thapliyal** for the course *ENPM700: Software Development for Robotics* at the University of Maryland.

## Project Dependencies
This package relies on the ROS Humble Hawksbill distribution. Please ensure it is installed beforehand.  
You can find the installation instructions [here](https://docs.ros.org/en/humble/Installation.html).
Ensure to have these installed:
```bash
pip install empy==3.3.4

# install ROS2 gazebo and turlebot3 packages
sudo apt -y install ros-humble-gazebo-ros-pkgs
sudo apt -y install ros-humble-turtlebot3*
sudo apt -y install ros-humble-turtlebot4-desktop

# install colcon tab completion 
sudo apt -y install python3-colcon-common-extensions
```

## Building the Project

First, you need to source the ROS 2 setup script to configure your environment:

```bash
source /opt/ros/humble/setup.bash
```

Next, create a new ROS2 workspace directory:

```bash
mkdir -p ~/gazebo_tutorials/src
```

Navigate to the source directory of your ROS2 workspace:

```bash
cd ~/gazebo_tutorials/src
```

Clone the package into your workspace:

```bash
git clone https://github.com/Apoorv-1009/my_gazebo_tutorials.git
```

Once the repository is cloned, change back to the workspace directory:

```bash
cd ~/gazebo_tutorials
```

Before building the package, install the necessary dependencies using rosdep:

```bash
rosdep install -i --from-path src --rosdistro humble -y
```

Now, compile the package with colcon:

```bash
colcon build 
```

After a successful build, source the package to make the executables available:

```bash
source ./install/setup.bash
```

## Running the Simulation

To start the Gazebo world with the turtlebot:
```bash
cd ~/gazebo_tutorials
ros2 launch walker gazebo_sim.launch.py
```
To record all topics except camera:
```bash
ros2 launch walker gazebo_sim.launch.py ros2_bag_record:=True
```
To start the turtlebot controller:
```bash
ros2 run walker turtlebot_controller 
```

### ROS2 Bag file 
To verify the topic messages of the recorded bag file, you can get its output with:
```bash
ros2 bag info recorded_topics
```
To run the bag file:
```bash
ros2 bag play recorded_topics
```
A sample bag file has been provided under `results/`

## Style Check Guidelines

To maintain code quality, you can perform style checks using cppcheck and cpplint. First, navigate to the package directory:
```bash
cd ~/gazebo_tutorials/src/walker
```

Run cppcheck for code analysis:
```bash
cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem $(find . -name "*.cpp" | grep -vE -e "^./build/") --check-config > results/cppcheck_output.txt
```

Run cpplint for style checking:
```bash
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/*.cpp > results/cpplint_output.txt
```

Run clang tidy for diagnosing and fixing common issues:
```bash
clang-tidy -extra-arg=-std=c++17 src/*.cpp
```

You can save the output with:
```bash
echo $? > results/clangtidy_output.txt
```

To directly format the code to Google style C++:
```bash
clang-format -style=Google -i src/*.cpp
```
