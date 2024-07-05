from enum import Enum

class Gait(Enum):
    T_POSE  = 't_pose'
    TRIPOD  = 'tripod'
    WAVE    = 'wave'
    RIPPLE  = 'ripple'


class Joint(Enum):
    base_to_coxa    = alpha = 0
    coxa_to_femur   = beta  = 1
    femur_to_tibia  = gamma = 2

    @classmethod
    def amount_per_leg(cls):
        return len(cls)


class Link(Enum):
    Coxa    = 0
    Femur   = 1
    Tibia   = 2
    Foot    = 3

    @property
    def length(self):
        '''Length in Meters'''
        return self.lengths()[self.value]

    @classmethod
    def lengths(cls):
        return [.072, .092, .162]


class Leg(Enum):
    LEFT_FRONT      = 1
    RIGHT_FRONT     = 2
    LEFT_MIDDLE     = 3
    RIGHT_MIDDLE    = 4
    LEFT_REAR       = 5
    RIGHT_REAR      = 6

    def frame_id(self, link: Link) -> str:
        return f'{self.value}_{link.name.lower()}_link'
    
    def joint_index(self, joint: Joint):
        '''Takes a joint and returns the index that joint on this leg has in a flat array like in the joint_states array'''
        return (self.value-1) * 3 + joint.value
    
    def motor_id(self, joint: Joint):
        '''Takes a joint and returns the motor_id that joint on this leg has'''
        return self.value * 10 + joint.value + 1

    @classmethod
    def amount(cls):
        '''Returns the amount of legs'''
        return len(cls)
