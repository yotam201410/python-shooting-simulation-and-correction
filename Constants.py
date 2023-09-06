import numpy as np
import math


class Constants:
    estimation_resolution = 0.01
    drag_constant = 0.47
    ball_radius = 0.12065
    ball_mass = 0.2676195
    gravitational_acceleration_constant = 9.8067
    gravitational_acceleration_matrix = np.array([0.0, 0.0, -gravitational_acceleration_constant])
    robot_height = 0.25
    robot_width = 0.25
    robot_length = 0.25
    robot_dimensions = [robot_width * 2, robot_length * 2, robot_height * 2]
    max_velocity = 8.0
    max_rotational_velocity = 360.0
    density_of_air = 1.1839
    joystick_drift_threshold = 0.1
    top_wheel_radius = 0.0762
    top_circumference = top_wheel_radius * 2 * math.pi
    bottom_wheel_radius = 0.0762
    bottom_circumference = bottom_wheel_radius * 2 * math.pi
    rpm_resolution = 50.0
    angle_resolution = 1
