import numpy as np
from motion_imitation.utilities import pose3d
from pybullet_utils  import transformations

URDF_FILENAME = "robot_urdf/KIST MAHRU-WL_w_Battery.urdf"

REF_POS_SCALE = 1
INIT_POS = np.array([0, 0, 1])
INIT_ROT = transformations.quaternion_from_euler(ai=0, aj=0, ak=-np.pi, axes="sxyz")

SIM_TOE_JOINT_IDS = [
    5, # left hand
    15, # left foot
    10, # right hand
    20 # right foot
]
SIM_HIP_JOINT_IDS = [2, 12, 7, 17]

SIM_TIP_JOINT_IDS = [ 4, 5, 13, 15, 9, 10, 18, 20,1,6] # * 어깨 추가시 1, 6 추가
SIM_ROOT_OFFSET = np.array([0, 0, 0])

SIM_TOE_OFFSET_LOCAL = [
    np.array([0.0, 0.0, 0.0]),
    np.array([0.0, 0.0, 0.0]),
    np.array([0.0, 0.0, 0.0]),
    np.array([0.0, 0.0, 0.0])
]
"""
SIM_TOE_OFFSET_LOCAL = [
    np.array([-0.02, 0.0, 0.0]),
    np.array([-0.02, 0.0, 0.01]),
    np.array([-0.02, 0.0, 0.0]),
    np.array([-0.02, 0.0, 0.01])
]
"""
"""
DEFAULT_JOINT_POSE = np.array([0, np.pi/2, 0, 0, -np.pi/4,
                                -np.pi/2, 0, 0, np.pi/4, 0,
                                  -np.pi/4, -np.pi/2, 0, 0,0,
                                  np.pi/4, np.pi/2,0,0,0,
                                  0,0])
"""
DEFAULT_JOINT_POSE = np.array([0, -0.35, 0, 0, -0.4,
                                0.35, 0, 0, 0.4, 0,
                                -0.2, 0.5, 0, 0,0,
                                0.2, -0.5, 0, 0, 0,
                                0, 0])

JOINT_DAMPING = [0.5, 0.05, 0.01,
                 0.5, 0.05, 0.01,
                 0.5, 0.05, 0.01,
                 0.5, 0.05, 0.01]


FORWARD_DIR_OFFSET = np.array([0, 0, 0.025])
