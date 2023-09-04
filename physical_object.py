from abc import ABC, abstractmethod, abstractproperty
import time
from typing import List

import meshcat
import numpy as np
from meshcat import Visualizer
from Constants import Constants
from state import State
import math

from target import Target


def multiple_list(vector_list: List[np.ndarray], number: float) -> List[np.ndarray]:
    return_list = []
    for vector in vector_list:
        return_list.append(vector*number)
    return return_list


def add_list(vector_list: List[np.ndarray], vector_list2: List[np.ndarray]) -> List[np.ndarray]:
    return_list = []
    for vector1, vector2 in zip(vector_list, vector_list2):
        return_list.append(vector1+vector2)
    return return_list


class PhysicalObject(ABC):
    def __init__(self, state: State, mass: float, before_before_state: State, estimation_resolution: float, radius: float):
        self.state = state
        self.before_before_state = before_before_state
        self.estimation_resolution = estimation_resolution
        self.mass = mass
        self.radius = radius

    def add_force(self, force: np.ndarray):
        self.state.sigma_forces += force

    def subtract_force(self, force: np.ndarray):
        self.add_force(-force)

    def calc(self):
        self.state.acceleration = self.state.sigma_forces / self.mass

    @abstractmethod
    def calc_forces(self):
        pass

    def simulate_object(self) -> List[State]:
        positions_array = []
        states_array = [State(self.state)]
        while self.state.position[2] > 0:
            before_state = State(self.state)
            self.calc_forces()
            self.calc()
            self.runge_kutta_approximation(before_state)
            positions_array.append(self.state.position)
            self.before_before_state = State(before_state)
            states_array.append(State(self.state))
        return states_array

    def simulate_object_step(self):
        before_state = State(self.state)
        self.calc_forces()
        self.calc()
        self.runge_kutta_approximation(before_state)
        self.before_before_state = State(before_state)
        return self.state

    def differential_equation(self, time: float, last_state: State) -> List[np.ndarray]:
        return [last_state.velocity, last_state.acceleration,
                last_state.acceleration - (self.before_before_state.acceleration * time)]

    def runge_kutta_approximation(self, before_state: State):
        # state = [before_state.position, before_state.velocity, before_state.acceleration]
        a = self.differential_equation(0, before_state)
        b = self.differential_equation(self.estimation_resolution * 0.5,
                                       before_state.add_vector(multiple_list(a, self.estimation_resolution * 0.5)))
        c = self.differential_equation(self.estimation_resolution * 0.5,
                                       before_state.add_vector(multiple_list(b, self.estimation_resolution * 0.5)))
        d = self.differential_equation(self.estimation_resolution,
                                       before_state.add_vector(multiple_list(c, self.estimation_resolution)))
        _sum = multiple_list(add_list(a, add_list(b, add_list(
            b, add_list(c, add_list(c, d))))), self.estimation_resolution/6.0)
        self.state.position += _sum[0]
        self.state.velocity += _sum[1]
        self.state.acceleration += _sum[2]

    def full_check(self, target: Target, angle: float, rpm: float, ratio: float, start_position: np.ndarray, starting_velocity: np.ndarray):
        top_rpm = rpm * (ratio - 1)if rpm > 0 else rpm
        bottom_rpm = rpm * (1-ratio) if ratio < 0.0 else rpm
        top_rps = top_rpm/60
        bottom_rps = bottom_rpm/60
        muzzele_velocity = (top_rps * Constants.top_circumference +
                            bottom_rps * Constants.bottom_circumference) / 2
        angle_rad = math.radians(angle)
        top_rads = top_rpm * 2 * math.pi/60
        bottom_rads = top_rpm * 2 * math.pi/60
        velocity_vector = np.array([0, muzzele_velocity * math.sin(
            angle_rad), muzzele_velocity*math.cos(angle_rad)]+starting_velocity)
        rotational_velocity_vector = np.array(
            [top_rads * Constants.top_wheel_radius - bottom_rads * Constants.bottom_wheel_radius/(2*self.radius), 0, 0])
        self.state.position = start_position
        self.state.velocity = velocity_vector
        self.state.rotational_velocity = rotational_velocity_vector
        states = self.simulate_object()
        return target.check(states)

    def full_distance_check(self, target: Target, angle: float, rpm: float, ratio: float, start_position: np.ndarray, starting_velocity: np.ndarray):
        top_rpm = rpm * (ratio - 1)if rpm > 0 else rpm
        bottom_rpm = rpm * (1-ratio) if ratio < 0.0 else rpm
        top_rps = top_rpm/60
        bottom_rps = bottom_rpm/60
        muzzele_velocity = (top_rps * Constants.top_circumference +
                            bottom_rps * Constants.bottom_circumference) / 2
        angle_rad = math.radians(angle)
        top_rads = top_rpm * 2 * math.pi/60
        bottom_rads = top_rpm * 2 * math.pi/60
        velocity_vector = np.array([0, muzzele_velocity * math.sin(
            angle_rad), muzzele_velocity*math.cos(angle_rad)]+starting_velocity)
        rotational_velocity_vector = np.array(
            [top_rads * Constants.top_wheel_radius - bottom_rads * Constants.bottom_wheel_radius/(2*self.radius), 0, 0])
        self.state.position = start_position
        self.state.velocity = velocity_vector
        self.state.rotational_velocity = rotational_velocity_vector
        states = self.simulate_object()
        return target.check_distance(states)

    def binary_smart_optimize_runge_kutta(self, target: Target, max_angle: float, min_angle: float, max_rpm: float, min_rpm: float, max_hub_distance: float, start_position: np.ndarray, start_velocity: np.ndarray):
        start = time.time()
        initial_angle = max_angle
        angle_increment = - Constants.angle_resolution
        current_angle = initial_angle
        initial_rotation_ratio = 0.0
        current_rotation_ratio = initial_rotation_ratio
        initial_rpm = max_rpm
        rpm_increment = Constants.rpm_resolution
        current_rpm = initial_rpm
        if max_hub_distance > 0:
            current_rotation_ratio = min(
                target.y_pos/max_hub_distance, 1.0) * -1.0
        attempts = 0
        best_top_rpm = -1.0
        best_bottom_rpm = -1.0
        best_angle = -1.0
        best_rpm = -1.0
        current_angle = initial_angle
        best_rpm = current_rpm
        best_angle = current_angle
        angles = [x for x in range(max_angle, min_angle, angle_increment)]
        while (len(angles) > 1):
            attempts+=1
            if current_rpm <= min_rpm:
                current_rpm = best_rpm
            fits_in_hub = False
            while (not fits_in_hub and current_rpm > min_rpm):
                current_rpm += rpm_increment
                fits_in_hub = self.full_check(
                    target, current_angle, current_rpm, current_rotation_ratio, start_position, start_velocity)
            if (self.full_check(target, current_angle, current_rpm, current_rotation_ratio, start_position, start_velocity)):
                best_rpm = current_rpm
                best_angle = current_angle
                best_top_rpm = best_rpm * \
                    (current_rotation_ratio -
                     1) if current_rotation_ratio > 0 else best_rpm
                best_bottom_rpm = best_rpm * \
                    (1-current_rotation_ratio) if current_rotation_ratio < 0 else best_rpm
            else:
                if (self.full_distance_check(target, current_angle, current_rpm, current_rotation_ratio, start_position, start_velocity) > 0):
                    angles = angles[len(angles)/2:len(angles)]
                else:
                    angles = angles[0:len(angles)/2]
                current_angle = angles[len(angles)/2]
            print(attempts)
        print(time.time()-start)
        return best_top_rpm, best_bottom_rpm, best_angle

    def get_target_threshold(self):
        tolerance = 0.1
        return self.radius * 4 + tolerance
