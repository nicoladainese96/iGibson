# This file contains all the low-level logic to implement the high-level actions common to all embodiments

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

from igibson.action_primitives.action_primitive_set_base import ActionPrimitiveError

class CustomSemanticActions()
    def __init__(self, task, scene, robot):
        self.task = task
        self.scene = scene
        self.robot = robot

