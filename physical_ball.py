import asyncio
import time
import numpy as np
from Constants import Constants
from typing import *
from ball import Ball
from meshcat import Visualizer
import meshcat
from threading import Thread
from visualizer import visualization


class PhysicalBall:
    def __init__(self, ball: Ball, visualizer: visualization, starting_velocity: np.ndarray):
        self.ball = ball
        meshcat.geometry.Sphere(Constants.ball_radius)
        self.vis = visualizer
        self.ID = self.vis.create_ball()
        self.ball.state.velocity = starting_velocity

    async def run(self):
        start = 0
        end = start+Constants.estimation_resolution
        self.vis.set_ball_position(
            self.ID, self.vis.robot_location + np.array([0, 0, Constants.robot_height]))
        self.ball.state.position = self.vis.robot_location + \
            np.array([0, 0, Constants.robot_height])
        self.ball.init_calculations()
        while self.ball.state.position[2] > 0:
            if time.time() >= end:
                start = time.time()
                self.ball.simulate_object_step()
                self.vis.set_ball_position(self.ID, self.ball.state.position)
                await asyncio.sleep(0)
                end = start + Constants.estimation_resolution

