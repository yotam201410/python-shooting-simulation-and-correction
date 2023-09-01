from visouliser import visualization
from controller import XboxController
from Constants import Constants
import time
import numpy as np
import math
dt  = 0
vis = visualization()
controller = XboxController()
def move_robot_pos(velocity:np.array,dt):
    vis.set_box_position(vis.robot_location+velocity*dt)
def move_robot_rotation(rotational_velocity:float,dt):
    vis.set_box_rotation(math.degrees(vis.robot_rotation)+rotational_velocity*dt)

while True:
    start = time.time()
    controls = controller.read()
    move_robot_pos(np.array([-controls[1],controls[0],0])*Constants.max_velocity,dt)
    move_robot_rotation(controls[2]*Constants.max_rotational_velocity,dt)
    dt = time.time() - start
    