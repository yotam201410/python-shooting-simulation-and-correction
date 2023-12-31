import threading
import meshcat

from ball import Ball
from physical_ball import PhysicalBall
from target import Target
from visualizer import visualization
from controller import XboxController
from Constants import Constants
import time
import numpy as np
import math
from typing import *
import asyncio
dt = 0
vis = visualization()
controller = XboxController()
loop = asyncio.new_event_loop()


def move_robot_pos(velocity: np.ndarray, dt: float):
    """
        moves the robot position by using the velocity of the robot
        :param velocity: the velocity of the robot in meters/second
        :param dt: the amount of time that passed since the last update
        """
    vis.set_box_position(vis.robot_location + velocity * dt)


def move_robot_rotation(rotational_velocity: float, dt: float):
    """
    moves the robot rotation by using the rotational velocity of the robot
    :param rotational_velocity: the rotational velocity of the robot in degrees/second
    :param dt: the amount of time that passed since the last update
    """
    vis.set_box_rotation(math.degrees(
        vis.robot_rotation) + rotational_velocity * dt)


def correct_drift(controls: List):
    for i in range(len(controls)):
        if abs(controls[i]) < Constants.joystick_drift_threshold:
            controls[i] = 0
    return controls


async def robot_movement():
    global dt, vis, controller
    while True:
        start = time.time()
        controls = correct_drift(controller.read())
        if controls[0] != 0 or controls[1] != 0:
            move_robot_pos(np.array([-controls[1], controls[0], 0])
                        * Constants.max_velocity, dt)
        if controls[2] !=0:
            move_robot_rotation(
                controls[2] * Constants.max_rotational_velocity, dt)
        await asyncio.sleep(0)
        dt = time.time() - start


async def shoot():
    global dt, vis, controller
    before = controller.read()
    while True:
        controls = correct_drift(controller.read())
        if controls[4] == 1 and before[4] == 0:
            ball = Ball(Constants.ball_mass, Constants.ball_radius,
                        Constants.drag_constant, Constants.estimation_resolution)
            # target = Target(np.array([0.0,4.0,2.7178]),np.array([1,1.22-ball.get_target_threshold(),0.05]),90,45,999,-999)
            # results = ball.binary_smart_optimize_runge_kutta(target,45,0,5000,1500,12,ball.state.position + np.array([0.0,4,0.25]),ball.state.velocity)
            # print(results)
            pball = PhysicalBall(ball, vis, np.array(
                [0.0, 0, 0]+ np.array([-controls[1], controls[0], 0])* Constants.max_velocity))
            ball.state.velocity = np.array([0,18,18.0])

            ball.state.rotational_velocity = np.array([99.2081890607,0.0,0.0])

            loop.create_task(pball.run())

            # pball = PhysicalBall(pball, vis, np.array([0.0,0.0,0.0]))

        before = controls
        await asyncio.sleep(0.1)
loop.create_task(robot_movement())
loop.create_task(shoot())
loop.run_forever()
