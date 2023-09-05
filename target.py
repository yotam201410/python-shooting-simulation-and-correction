import numpy as np
from typing import List
from state import State
import math
import util


class Segment:
    def __init__(self, lx: float, ly: float, cx: float, cy: float):
        self.LX = lx
        self.LY = ly
        self.CX = cx
        self.CY = cy


class Target:
    def __init__(self, position: np.ndarray, size: np.ndarray, maximum_entry_angle: float, minimum_entry_angle: float, maximum_entry_velocity: float, minimum_entry_velocity: float):
        self.y_pos = position[1]
        self.z_pos = position[2]
        self.y_size = size[1]
        self.z_size = size[2]
        self.maximum_entry_angle = maximum_entry_angle
        self.minimum_entry_angle = minimum_entry_angle
        self.minimum_entry_velocity = minimum_entry_velocity
        self.maximum_entry_velocity = maximum_entry_velocity

    def set_distance(self, distance):
        self.y = distance

    def line_line(self, startA: np.ndarray, endA: np.ndarray, startB: np.ndarray, endB: np.ndarray) -> bool:
        x1 = startA[0]
        y1 = startA[1]
        x2 = endA[0]
        y2 = endA[1]
        x3 = startB[0]
        y3 = startB[1]
        x4 = endB[0]
        y4 = endB[1]
        uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / \
            ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1) + 0.001)
        uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / \
            ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1) + 0.001)
        if (uA >= 0 and uA <= 1 and uB >= 0 and uB <= 1):
            return True
        return False

    def check(self, states: List[State])->bool:
        if (len(states) < 5):
            return False
        segment = Segment(0.0, 0.0, 0.0, 0.0)
        x_max = self.y_pos + (self.y_size / 2.0)
        x_min = self.y_pos - (self.y_size / 2.0)
        y_max = self.z_pos + (self.z_size / 2.0)
        y_min = self.z_pos - (self.z_size / 2.0)
        for i in range(len(states)):
            segment.LX = segment.CX
            segment.LY = segment.CY
            segment.CX = states[i].position[1]
            segment.CY = states[i].position[2]
            angle_deg = -math.degrees(math.atan2(
                    segment.CY - segment.LY, segment.CX - segment.LX))
            velocity = util.get_vector_magnitude(
                np.array[(segment.CX - segment.LX, segment.CY - segment.LY)])
            if self.line_line(np.array([segment.LX, segment.LY]),  np.array([segment.CX, segment.CY]), np.array([x_min, y_max]), np.array([x_max, y_max])) or self.line_line(np.array([segment.LX, segment.LY]), np.array([segment.CX, segment.CY]), np.array([x_min, y_min]), np.array([x_max, y_min])) or self.line_line(np.array([segment.LX, segment.LY]), np.array([segment.CX, segment.CY]), np.array([x_min, y_max]), np.array([x_min, y_min])) or self.line_line(np.array([segment.LX, segment.LY]), np.array([segment.CX, segment.CY]), np.array([x_max, y_max]), np.array([x_max, y_min])):
                if angle_deg >= self.minimum_entry_angle and angle_deg <= self.maximum_entry_angle:
                    if velocity >= self.minimum_entry_velocity and velocity <= self.maximum_entry_velocity:
                        return True
        return False
    def check_distance(self,states: List[State])->float:
        if (len(states) < 5):
            return False
        segment = Segment(0.0, 0.0, 0.0, 0.0)
        x_max = self.y_pos + (self.y_size / 2.0)
        x_min = self.y_pos - (self.y_size / 2.0)
        y_max = self.z_pos + (self.z_size / 2.0)
        y_min = self.z_pos - (self.z_size / 2.0)
        for i in range(len(states)):
            print(states[i])
            segment.LX = segment.CX
            segment.LY = segment.CY
            segment.CX = float(states[i].position[1])
            segment.CY = float(states[i].position[2])
            angle_deg = -math.degrees(math.atan2(segment.CY - segment.LY, segment.CX - segment.LX))
            velocity = util.get_vector_magnitude(
                np.array([segment.CX - segment.LX, segment.CY - segment.LY]))
            # print(angle_deg,velocity)
            if angle_deg >= self.minimum_entry_angle and angle_deg <= self.maximum_entry_angle:
                    if velocity >= self.minimum_entry_velocity and velocity <= self.maximum_entry_velocity:
                        return ((segment.LX + segment.CX) / 2.0) - self.y_pos