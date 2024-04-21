# Multi-Agent Reinforcement Learning for Turtlebots
This repo contains ROS2 packages for **Multi-Agent Reinforcement Learning (MARL)** development on simulated and real Turtlebots. 
The main motivation behind this repo is to implement multi-agent exploration on turtlebots using RL.
This includes a custom sim environment built from scratch and uses `slam_toolbox` for SLAM.
Many of the packages here that handle the integration of simulation, sensing, controls etc. have been adapted from my other repo where I built all of this from scratch to implement my version of **Extended Kalman-Filter feature-based SLAM** on a real turtlebot: https://github.com/ME495-Navigation/slam-project-GogiPuttar

# Package List
This repository consists of several ROS2 packages

## General Purpose
- `nuturtle_description` - 3D models of the nuturtle for simulation and visualization.
- `turtlelib` - CMake Library with functions for control and SLAM of a differential drive robot

## EKF SLAM specific
- `nusim` - Loads the world consisting of the robot, arena and obstacles, and simulates the robot motion, control and sensors, akin to a real robot.
- `nuturtle_control` - Handles the control and sensing related to the wheels of the robot
- `nuslam` - Performs EKF SLAM using the turtlelbot

## MARL specific
- `multisim` - Generates and simulates exploration environments with multiple robots. (TODO)
- `multicontrol` - Controls multiple robots. (TODO)
- `multislam` - Uses `slam_toolbox` for multi-agent centralized SLAM. (TODO)
- `multiRL` - Uses RL for multi-agent exploration. (TODO)
