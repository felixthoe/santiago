import numpy as np
import sympy as sp

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np

data = np.load("/home/uwxvp/ws_Aufstehhilfe/santiago/model_data/aufstehhilfe_model_LUTs.npz")
for k in data.files:
    print(k, data[k].shape, data[k].dtype)