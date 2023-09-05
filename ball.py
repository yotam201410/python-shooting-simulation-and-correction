import math
from abc import ABC

import meshcat
import numpy as np

import util
from Constants import Constants
from physical_object import PhysicalObject
from state import State


class Ball(PhysicalObject, ABC):
    def __init__(self,mass: float, radius: float, drag_constant: float,
                 estimation_resolution: float):
        super().__init__(State(), mass, State(), estimation_resolution,radius)
        self.lift_coefficient = 0
        self.drag_constant = drag_constant
        self.cross_section_area = radius**2 * math.pi
        self.moment_of_inertia = mass * radius ** 2

    def init_calculations(self):
        if util.get_vector_magnitude(self.state.velocity)!=0:
            self.lift_coefficient = self.radius * util.get_vector_magnitude(
                self.state.rotational_velocity) / util.get_vector_magnitude(self.state.velocity)

    def calc_forces(self):
        self.state.sigma_forces = np.array([0.0, 0.0, 0.0])
        velocity_direction = util.get_vector_direction(self.state.velocity)
        velocity_magnitude = util.get_vector_magnitude(self.state.velocity)
        self.add_force(Constants.gravitational_acceleration_matrix*self.mass)
        self.add_force(
            -1 * velocity_direction * 0.5 * self.drag_constant * Constants.density_of_air * self.cross_section_area *(velocity_magnitude **2))
        if abs(util.get_vector_magnitude(self.state.rotational_velocity)) > 0:
            axis_of_rotation = util.get_vector_direction(self.state.rotational_velocity)
            self.add_force(np.cross(velocity_direction,
                                    axis_of_rotation) * 0.5 * self.lift_coefficient * Constants.density_of_air *self.cross_section_area* (velocity_magnitude ** 2))
