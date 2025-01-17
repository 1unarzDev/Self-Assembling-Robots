import numpy as np
from rrt_star import get_length_ratio, get_orientation

DELTA_T = 0.1

RPM = 300
WHEEL_DIAMETER = 59

MAX_WHEEL_ROT_SPEED_RAD = RPM * np.pi / 30
MIN_WHEEL_ROT_SPEED_RAD = -MAX_WHEEL_ROT_SPEED_RAD

MAX_LINEAR_VELOCITY = 100
MIN_LINEAR_VELOCITY = -MAX_LINEAR_VELOCITY
TECHNICAL_MAX_LINEAR_VELOCITY = WHEEL_DIAMETER * get_length_ratio() * np.pi * RPM / 60

MPC_HORIZON = 5