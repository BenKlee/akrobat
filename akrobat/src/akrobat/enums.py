from enum import IntEnum

class Gait(IntEnum):
    RESET   = -1
    TRIPOD  = 0
    WAVE    = 1
    RIPPLE  = 2


class Joint(IntEnum):
    base_to_coxa    = alpha = 0
    coxa_to_femur   = beta  = 1
    femur_to_tibia  = gamma = 2


class Leg(IntEnum):
    LEFT_FRONT      = 0
    RIGHT_FRONT     = 1
    LEFT_MIDDLE     = 2
    RIGHT_MIDDLE    = 3
    LEFT_REAR       = 4
    RIGHT_REAR      = 5


    def joint_index(self, joint: Joint):
        '''Takes a joint and returns the index that joint on this leg has in a flat array like the joint_states array'''
        return self * 3 + joint


    @classmethod
    def amount(leg_enum):
        '''Returns the amount of legs'''
        return len(leg_enum)