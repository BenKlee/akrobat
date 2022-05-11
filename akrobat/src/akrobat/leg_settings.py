from dataclasses import dataclass


@dataclass
class LegSettings:
    roll_over:              float
    rotation_of_coxa:       float

    body_const_x:           float
    body_const_y:           float
    body_const_z:           float

    joint_initial_alpha:    float
    joint_initial_beta:     float
    joint_initial_gamma:    float

    minimum_alpha:          float
    minimum_beta:           float
    minimum_gamma:          float

    maximum_alpha:          float
    maximum_beta:           float
    maximum_gamma:          float