from abc import ABC, abstractmethod, abstractproperty
from typing import List

import meshcat
import numpy as np
from meshcat import Visualizer

from state import State


def multiple_list(vector_list: List[np.ndarray], number: float) -> List[np.ndarray]:
    return_list = []
    for vector in vector_list:
        return_list.append(vector*number)
    return return_list


def add_list(vector_list: List[np.ndarray], vector_list2: List[np.ndarray]) -> List[np.ndarray]:
    return_list = []
    for vector1,vector2 in zip(vector_list, vector_list2):
        return_list.append(vector1+vector2)
    return return_list


class PhysicalObject(ABC):
    def __init__(self, state: State, mass: float, before_before_state: State, estimation_resolution: float):
        self.state = state
        self.before_before_state = before_before_state
        self.estimation_resolution = estimation_resolution
        self.mass = mass



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
        _sum = multiple_list(add_list(a, add_list(b, add_list(b, add_list(c,add_list(c, d))))), self.estimation_resolution/6.0)
        self.state.position += _sum[0]
        self.state.velocity += _sum[1]
        self.state.acceleration += _sum[2]
