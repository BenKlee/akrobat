from collections import namedtuple
from enum import Enum
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from numpy import radians
from math import cos, acos, pi, sin, sqrt, degrees, atan2


class Gait(Enum):
    TRIPOD  = 0
    WAVE    = 1
    RIPPLE  = 2


class Joint(Enum):
    base_to_coxa    = alpha = 0
    coxa_to_femur   = beta  = 1
    femur_to_tibia  = gamma = 2


class Leg(Enum):
    LEFT_FRONT      = 1
    RIGHT_FRONT     = 2
    LEFT_MIDDLE     = 3
    RIGHT_MIDDLE    = 4
    LEFT_REAR       = 5
    RIGHT_REAR      = 6

    def joint_index(self, joint: Joint):
        '''Takes a joint and returns the index that joint on this leg has in a flat array like in the joint_states array'''
        return (self-1) * 3 + joint
    
    def motor_id(self, joint: Joint):
        '''Takes a joint and returns the motor_id that joint on this leg has'''
        return self * 10 + joint

    @classmethod
    def amount(leg_enum):
        '''Returns the amount of legs'''
        return len(leg_enum)

class Link(Enum):
    Coxa    = 0
    Femur   = 1
    Tibia   = 2

    @property
    def length(self):
        return self.lengths()[self.value]

    @classmethod
    def lengths(cls):
        return [72, 92, 162]


Coordinate = namedtuple('Coordinate', ['x', 'y', 'z'])

class Akrobat(Node):

    def __init__(self, publish_frequency_Hz: int = 20, initial_gait: Gait = Gait.TRIPOD) -> None:
        super().__init__('minimal_publisher')

        self.__publish_frequency_Hz = publish_frequency_Hz
        self.__current_gait = initial_gait
        self.__gait_speed = 1
        self.__goal_positions = [0.,-0.5,-0.5] * Leg.amount()

        self.__state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(1/self.__publish_frequency_Hz, self.__publish_state)

        self.run_gait(self.__current_gait)

    def __publish_state(self):
        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'akrobat_link'

        msg.name = [f'm{leg}{joint}' for leg in range(1,7,1) for joint in range(1,4,1)]
        msg.position = self.__goal_positions
        msg.velocity = [0.] * 6*3
        msg.effort = [0.] * 6*3

        self.__state_publisher.publish(msg)


    def __inverse_kinematics(self, coordinate: Coordinate):
        '''
        Calculates the inverse kinematics for a specific leg

                Parameters:
                        coordinate (Coordinate):    the goal coordinate (x, y, z)

                Returns:
                        alpha (float):              the alpha angle in radians
                        beta (float):               the beta angle in radians
                        gamma (float):              the gamma angle in radians
        '''

        x, y, z = coordinate

        R = sqrt(y**2 + x**2)
        alpha = atan2(y, x)
        L = sqrt(((R - Link.Coxa.length)**2) + z**2)
        beta1 = atan2(z, (R - Link.Coxa.length))
        beta2 = acos(0.5 * ((Link.Femur.length**2) + (L**2) - (Link.Tibia.length**2)) / (L * Link.Femur.length));

        gamma = acos(0.5 * ((Link.Tibia.length**2) + (Link.Femur.length**2) - (L**2)) / (Link.Femur.length * Link.Tibia.length));

        if not self.rolled_over:
            beta = beta1 + beta2
            gamma = gamma - radians(180)
        else:
            beta = beta1 - beta2
            gamma = radians(180) - gamma

        return alpha, beta, gamma

    def set_leg_goal_coordiante(self, leg: Leg, coordinate: Coordinate):
        '''
        Calculates the joint angles for the given coordinate and leg using inverse kinematics.
        Sets the joint angles as goal positions for the next publish.
        '''
        alpha, beta, gamma = self.__inverse_kinematics(coordinate)

        # TODO coordinate transform to leg coordinate system

        self.__goal_positions[leg.joint_index(Joint.alpha)] = alpha
        self.__goal_positions[leg.joint_index(Joint.beta)]  = beta
        self.__goal_positions[leg.joint_index(Joint.gamma)] = gamma

    def run_gait(self, gait: Gait):
        self.__goal_positions = [0., -0.5, -0.5] * 6

def main(args=None):
    rclpy.init(args=args)

    akrobat = Akrobat()

    rclpy.spin(akrobat)

    akrobat.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
