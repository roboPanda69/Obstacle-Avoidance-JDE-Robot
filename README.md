# Obstacle Avoidance Robot

This project implements an obstacle avoidance algorithm for an autonomous robot using Python. The robot navigates towards a target while avoiding obstacles detected by a laser sensor.

## Overview

The robot uses a combination of target and obstacle vectors to determine its movement direction. The target vector guides the robot towards its goal, while the obstacle vector steers it away from obstacles. The robot adjusts its speed dynamically based on the proximity of obstacles.

### Key Features

- **Dynamic Speed Adjustment**: The robot slows down as it approaches obstacles.
- **Smooth Angular Velocity**: The robot avoids sharp turns, resulting in smoother movement.
- **Real-Time Obstacle Detection**: Laser data is continuously processed to detect obstacles.

## Code Explanation

The main Python script (`obstacle_avoidance.py`) contains the following key functions:

- `parse_laser_data(laser_data)`: Parses the laser sensor data to calculate an obstacle vector.
- `absolute2relative(x_abs, y_abs, robotx, roboty, robott)`: Converts absolute coordinates to relative coordinates based on the robot's current position and orientation.
- `while True` loop: Continuously calculates the target and obstacle vectors, adjusts the robot's speed and angular velocity, and visualizes the forces using the GUI.

## Getting Started

### Prerequisites

- Python 3.x
- JdeRobot's Robotics Academy installed

### Running the Code

1. Clone the repository:
   ```bash
   git clone https://github.com/roboPanda69/Obstacle-Avoidance-JDE-Robot.git

