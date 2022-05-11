# from math import cos, sin
import numpy as np

def from_homogenous(array: np.array) -> np.array:
    return array[:-1]/array[-1]

def to_homogenous(array: np.array) -> np.array:
    return np.append(array, 1)

# def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> np.array:
#     cos_yaw      = cos(yaw/2)
#     sin_yaw      = sin(yaw/2)
#     cos_pitch    = cos(pitch/2)
#     sin_pitch    = sin(pitch/2)
#     cos_roll     = cos(roll/2)
#     sin_roll     = sin(roll/2)

#     return np.array((sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw,
#         cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw,
#         cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw,
        # cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw))