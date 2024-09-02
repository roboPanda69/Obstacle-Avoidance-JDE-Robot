import GUI
import HAL
import math
import numpy as np

k_obs = -0.5
k_target = 0.1
k_final = 0.3
k_angle = 0.1
safe_distance = 1.0  # Distance at which the robot starts slowing down
max_speed = 4  # Maximum speed of the robot

currentTarget = GUI.getNextTarget()

def parse_laser_data(laser_data):
    obstacle_vector = [0,0]
    i = 0
    while (i < 180):
        dist = laser_data.values[i]
        if dist > 10:
            dist = 10
        # Use Gaussian function for smoother influence
        weight = k_obs * math.exp(-dist) * (1/(dist ** 2))
        angle = math.radians(i-90)  # because the front of the robot is -90 degrees
        obstacle_vector[0] += weight * math.cos(angle)
        obstacle_vector[1] += weight * math.sin(angle)
        i += 1
    
    return obstacle_vector
    
def absolute2relative(x_abs, y_abs, robotx, roboty, robott):
    # Convert to relatives
    dx = x_abs - robotx
    dy = y_abs - roboty

    # Rotate with current angle
    x_rel = dx * math.cos(-robott) - dy * math.sin(-robott)
    y_rel = dx * math.sin(-robott) + dy * math.cos(-robott)

    return [x_rel, y_rel]

while True:
    # Target Vector
    targetx = currentTarget.getPose().x
    targety = currentTarget.getPose().y
    robotx = HAL.getPose3d().x
    roboty = HAL.getPose3d().y
    robott = HAL.getPose3d().yaw
    
    target_vector = absolute2relative(targetx, targety, robotx, roboty, robott)
    target_vector = [k_target * target_vector[0], k_target * target_vector[1]]
    
    if target_vector[0] < 0.1:
        currentTarget.setReached(True)
        currentTarget = GUI.getNextTarget()
        
    # Obstacle Vector
    laser_data = HAL.getLaserData()
    obstacle_vector = parse_laser_data(laser_data)
    
    # Final Vector
    final_vector = [0,0]
    final_vector[0] = k_final * (target_vector[0] + obstacle_vector[0])
    final_vector[1] = k_final * (target_vector[1] + obstacle_vector[1])

    
    # Smooth the angular velocity to reduce sharp turns
    angular_velocity = k_angle * final_vector[1]
    
    # Dynamic speed adjustment based on obstacle proximity
    min_dist = min(laser_data.values)
    speed = max_speed if min_dist > safe_distance else max_speed * (min_dist / safe_distance)
    
    HAL.setV(speed)
    HAL.setW(angular_velocity)
    
    GUI.showForces(target_vector, obstacle_vector, final_vector)