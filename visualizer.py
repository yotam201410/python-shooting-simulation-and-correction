import math
import meshcat
import numpy as np
from Constants import Constants
class visualization:
    def __init__(self):
        self.vis = meshcat.Visualizer()
        self.vis.open()
        self.n=0
        self.robot_location = np.array([0,0,0])
        self.robot_rotation = 0
        self.vis["hub"].set_object(meshcat.geometry.StlMeshGeometry.from_file("TE-22100 v1.stl"))
        self.vis["hub"].set_transform(meshcat.transformations.scale_matrix(0.001))
        self.create_box()
    def create_box(self):
        box = meshcat.geometry.Box(Constants.robot_dimensions)
        self.vis["box"].set_object(box)
    def create_ball(self):
        ball = meshcat.geometry.Sphere(0.2413)
        self.vis[f'ball{self.n}'].set_object(ball)
        self.n+=1
    def set_box_position(self,pos):
        self.robot_location =pos
        self.vis['box'].set_transform(meshcat.transformations.translation_matrix(self.robot_location).dot(meshcat.transformations.rotation_matrix(self.robot_rotation,[0,0,1])))

    def set_box_rotation(self,rotation):
        self.robot_rotation = rotation
        self.robot_rotation = math.radians(rotation+90)
        self.vis["box"].set_transform(meshcat.transformations.translation_matrix(self.robot_location).dot(meshcat.transformations.rotation_matrix(self.robot_rotation,[0,0,1])))

    def set_ball_position(self,ID,pos):
        self.vis[f'ball{ID}'].set_transform(meshcat.transformations.translation_matrix(pos))



