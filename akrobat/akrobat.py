import asyncio
from collections import namedtuple
from enum import Enum
import time
import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import JointState
from numpy import radians
from math import cos, acos, pi, sin, sqrt, degrees, atan2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Point
from tf2_geometry_msgs import PointStamped


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

    def link_frame_name(self, joint: Joint) -> str:
        return f'{self.value}_{joint.name.lower()}_link'

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


CoordinateBase = namedtuple('Coordinate', ['x', 'y', 'z'])

class Coordinate(CoordinateBase):
    
    @classmethod
    def from_point(cls, point: Point) -> 'Coordinate':
        return cls(point.x, point.y, point.z)

    def to_point(self) -> Point:
        point = Point()
        point.x, point.y, point.z = [float(val) for val in self]
        return point


class Akrobat(Node):

    def __init__(self, node_name: str, publish_frequency_Hz: int = 20, initial_gait: Gait = Gait.TRIPOD) -> None:
        super().__init__(node_name)

        self.__publish_frequency_Hz = publish_frequency_Hz
        self.__current_gait = initial_gait
        self.__gait_speed = 1

        self.__current_positions = None
        '''The most up-to-date positions of the robot joints'''

        self.create_subscription(JointState, 'joint_states', self.__update_positions, 10)

        self.__goal_positions = [0.,-0.5,-0.5] * Leg.amount()
        '''The desired positions of the robot joints'''

        self.__robot_frame_name = 'akrobat_link' # TODO get from URDF?

        self.__state_publisher = self.create_publisher(JointState, 'goal_joint_states', 10)
        self.create_timer(1/self.__publish_frequency_Hz, self.__publish_state)

        self.__tf_buffer = Buffer()

        asyncio.run(self.run(self.__current_gait))

    def __publish_state(self):
        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.__robot_frame_name

        msg.name = [f'm{leg}{joint}' for leg in range(1,7,1) for joint in range(1,4,1)]
        msg.position = self.__goal_positions
        msg.velocity = [0.] * 6*3
        msg.effort = [0.] * 6*3

        self.__state_publisher.publish(msg)

    def __update_positions(self, joint_state: JointState):
        self.__current_positions = joint_state.position

    def __inverse_kinematics(self, coordinate: Coordinate) -> Coordinate:
        '''
        Calculates the inverse kinematics for one leg

                Parameters:
                        coordinate (Coordinate):    the goal coordinate in the leg's coordinate system (x, y, z)

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

        # if not self.rolled_over:
        beta = beta1 + beta2
        gamma = gamma - radians(180)
        # else:
        #     beta = beta1 - beta2
        #     gamma = radians(180) - gamma

        return alpha, beta, gamma

    def set_leg_goal_coordiante(self, leg: Leg, coordinate: Coordinate) -> None:
        '''
        Calculates the joint angles for the given coordinate and leg using inverse kinematics.
        Sets the joint angles as goal positions for the next publish.
        '''
        leg_coordinate = self.__akrobat_to_leg_coordinate_system(leg, coordinate)

        alpha, beta, gamma = self.__inverse_kinematics(leg_coordinate)

        self.__goal_positions[leg.joint_index(Joint.alpha)] = alpha
        self.__goal_positions[leg.joint_index(Joint.beta)]  = beta
        self.__goal_positions[leg.joint_index(Joint.gamma)] = gamma

    def __leg_to_akrobat_coordinate_system(self, leg: Leg, coordinate: Coordinate) -> Coordinate:
        point_message = PointStamped()
        point_message.header.frame_id = leg.link_frame_name(Link.Coxa)
        point_message.header.stamp = self.get_clock().now().to_msg()
        point_message.point = coordinate.to_point()
        new_point = self.__tf_buffer.transform(point_message, self.__robot_frame_name)
        return Coordinate.from_point(new_point)


    def __akrobat_to_leg_coordinate_system(self, leg: Leg, coordinate: Coordinate) -> Coordinate:
        point_message = PointStamped()
        point_message.header.frame_id = self.__robot_frame_name
        point_message.header.stamp = self.get_clock().now().to_msg()
        point_message.point = coordinate.to_point()
        new_point = self.__tf_buffer.transform(point_message, leg.link_frame_name(Link.Coxa))
        return Coordinate.from_point(new_point)


    async def run(self, initial_gait: Gait):
        while self.__current_positions is None:
            self.get_logger().warn('not received any joint_states from robot/sim yet')
            time.sleep(.5)

        
        # for leg in Leg:
        #     self.set_leg_goal_coordiante(leg, Coordinate(0, .30 if leg.value % 2 == 0 else -.30, leg.value/10 - .3))

        self.__goal_positions = [0., -0.5, -0.5]*6


def main(args=None):
    rclpy.init(args=args)

    akrobat = Akrobat(node_name='akrobat')

    rclpy.spin(akrobat)

    akrobat.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
