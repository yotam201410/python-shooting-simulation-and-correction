import math
import meshcat
import numpy as np
from Constants import Constants


class visualization:
    def __init__(self):
        self.vis= meshcat.Visualizer().open()
        self.n = 0
        self.robot_location = np.array([0, 0, 0])  # robot position
        self.robot_rotation = 0  # robot angle in radians
        self.vis["hub"].set_object(meshcat.geometry.StlMeshGeometry.from_file("TE-22100 v1.stl"))
        self.vis["hub"].set_transform(meshcat.transformations.scale_matrix(0.001))
        self.create_box()

    def create_box(self):
        """
        creates a box or the robot
                :rtype: None

        """
        box = meshcat.geometry.Box(Constants.robot_dimensions)
        self.vis["box"].set_object(box)

    def create_ball(self):
        """
        creates a ball and gives it an index
                :rtype: None

        """
        ball = meshcat.geometry.Sphere(Constants.ball_radius)
        self.vis[f'ball{self.n}'].set_object(ball)
        self.n += 1
        return self.n-1

    def set_box_position(self, pos: np.ndarray):
        """
        changes the position of the box
        :rtype: None
        :param pos: the position of the box in meters
        """
        self.robot_location = pos
        self.vis['box'].set_transform(meshcat.transformations.translation_matrix(self.robot_location).dot(
            meshcat.transformations.rotation_matrix(self.robot_rotation, [0, 0, 1])))

    def set_box_rotation(self, rotation):
        """
        changes the rotation of the box
        :param rotation: the rotation of the box in degrees
        """
        self.robot_rotation = rotation
        self.robot_rotation = math.radians(rotation + 90)
        self.vis["box"].set_transform(meshcat.transformations.translation_matrix(self.robot_location).dot(
            meshcat.transformations.rotation_matrix(self.robot_rotation, [0, 0, 1])))

    def set_ball_position(self, ID: int, pos):
        """
        changes the position of the ball
        :param ID:  ID
        :param pos: the ball position in meters
        """
        self.vis[f'ball{ID}'].set_transform(meshcat.transformations.translation_matrix(pos))
   