from __future__ import annotations

from typing import List

import numpy as np


class State:
    def __init__(self, state: State = None):
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0, 0.0])
        self.sigma_forces = np.array([0.0, 0.0, 0.0])
        self.rotational_velocity = np.array([0.0, 0.0, 0.0])
        self.rotational_acceleration = np.array([0.0, 0.0, 0.0])
        self.jerk = np.array([0.0, 0.0, 0.0])
        if state is not None:
            self.position = state.position
            self.velocity = state.velocity
            self.acceleration = state.acceleration
            self.sigma_forces = state.sigma_forces
            self.rotational_acceleration = state.rotational_acceleration
            self.rotational_velocity = state.rotational_velocity
            self.jerk = state.jerk

    def add_vector(self, vector_list: List[np.ndarray]) -> State:
        return_state = State(self)
        return_state.position += vector_list[0]
        return_state.velocity += vector_list[1]
        return_state.acceleration += vector_list[2]
        return return_state

    def __str__(self):
        return f"{self.position} {self.velocity} {self.acceleration} {self.jerk} {self.rotational_velocity}"
