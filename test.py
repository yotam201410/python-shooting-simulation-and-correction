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
ball = Ball(Constants.ball_mass, Constants.ball_radius,
                        Constants.drag_constant, Constants.estimation_resolution)
target = Target(np.array([0.0,4.0,2.7178]),np.array([1,1.22-ball.get_target_threshold(),0.05]),90,45,999,-999)
print(ball.full_distance_check(target,70,5000,0.5,np.array([0.0,4.0,0.1]),np.array([0.,0.,0.])))
# ball.state.position = np.array([0,0,0.25])
# ball.state.velocity = np.array([0,18,18.0])
# ball.state.rotational_velocity = np.array([-99.2081890607,0.0,0.0])
# states = ball.simulate_object()
# for i in states:
#     print(i)