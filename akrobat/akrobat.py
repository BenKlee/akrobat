import asyncio
from collections import namedtuple
from enum import Enum
import time
import rclpy
import rclpy.duration
from rclpy.node import Node
import rclpy.time
from numpy import radians
from math import cos, acos, pi, sin, sqrt, degrees, atan2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros

from std_msgs.msg import Float64MultiArray, Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point, Transform
from tf2_geometry_msgs import PointStamped
from sensor_msgs.msg import JointState

class Gait(Enum):
    T_POSE  = 0
    TRIPOD  = 1
    WAVE    = 2
    RIPPLE  = 3


class Joint(Enum):
    base_to_coxa    = alpha = 0
    coxa_to_femur   = beta  = 1
    femur_to_tibia  = gamma = 2


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
    def amount(leg_enum):
        '''Returns the amount of legs'''
        return len(leg_enum)


CoordinateBase = namedtuple('Coordinate', ['x', 'y', 'z'])

class Coordinate(CoordinateBase):
    '''3-dimensional Coordinate in Meters'''

    
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
        self.__gait_update_frquency_Hz = 10
        self.__gait_speed_modifier = 1
        self.__robot_frame_name = 'akrobat_link' # TODO get from URDF?

        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self)

        self.__current_positions = None
        '''The most up-to-date positions of the robot joints'''
        self.__goal_positions = [0.] * 3 * Leg.amount()
        '''The desired positions of the robot joints'''

        self.__gait_step = 0
        self.__gait_granularity = 15

        self.__state_publisher = self.create_publisher(Float64MultiArray, 'joint_position_controller/commands', 10)
        self.create_subscription(JointState, 'joint_states', self.__update_positions, 10)
        self.create_timer(1/self.__publish_frequency_Hz, self.__publish_state)
        self.__run_timer = self.create_timer((1/self.__gait_update_frquency_Hz)/self.__gait_speed_modifier, self.run)
        self.create_subscription(PointStamped, 'goal_point', self.__set_goal_point, 10)

    @property
    def gait_speed_modifier(self):
        '''
        How fast or slow to run the current gait.
        '''
        return self.__gait_speed_modifier
    
    @gait_speed_modifier.setter
    def gait_speed_modifier(self, value):
        # TODO handle value == 0
        self.__gait_speed_modifier = value
        period_sec = (60/self.__gait_update_frquency_Hz)/self.__gait_speed_modifier
        period_ns = period_sec / (1000**3)
        self.__run_timer.timer_period_ns = period_ns

    def __publish_state(self):
        msg = Float64MultiArray()
        msg.data = self.__goal_positions[:6] # TODO change back to all positions
        self.__state_publisher.publish(msg)

    def __update_positions(self, joint_state: JointState):
        self.__current_positions = joint_state.position

    def __inverse_kinematics(self, point: Point, leg: Leg) -> tuple[float]:
        '''
        Calculates the inverse kinematics for one leg

                Parameters:
                        point (Point):    the goal coordinate in the leg's coordinate system (x, y, z)

                Returns:
                        alpha (float):              the alpha angle in radians
                        beta (float):               the beta angle in radians
                        gamma (float):              the gamma angle in radians
        '''

        X, Y, Z = point.x , point.y, -point.z

        A, B, C = Link.Coxa.length, Link.Femur.length, Link.Tibia.length

        try:
            alpha = atan2(X, Y)

            R = sqrt(Y**2 + X**2)
            S = R - A
            L = sqrt(S**2 + Z**2)

            beta1 = atan2(Z, S)
            beta2 = acos((B**2 + L**2 - C**2) / (2 * L * B))
            beta = beta1 - beta2

            gamma = acos((C**2 + B**2 - L**2) / (2 * B * C))
            gamma = radians(180) - gamma

            self.get_logger().debug(f'calculated joint positions {alpha=}, {beta=}, {gamma=} from {point=}')

            # invert for left/right legs
            return alpha, beta, gamma
        except ValueError:
            self.get_logger().error(f'math domain error during inverse kinematics calculation with {point=}')
            return .0,.0,.0

    def __set_goal_point(self, msg: PointStamped):
        if msg.header.frame_id == self.__robot_frame_name:
            self.set_leg_goal_coordiante(Leg.RIGHT_REAR, msg)
        else:
            for leg in Leg:
                if leg.value == int(msg.header.frame_id[0]):
                    self.set_leg_goal_coordiante(leg, msg)

    def set_leg_goal_coordiante(self, leg: Leg, point_stamped: PointStamped) -> None:
        '''
        Calculates the joint angles for the given coordinate and leg using inverse kinematics.
        Sets the joint angles as goal positions for the next publish.
        '''
        target_frame = leg.frame_id(Link.Coxa)

        self.get_logger().debug(f'requested point {point_stamped.point} in frame {point_stamped.header.frame_id}')

        point_stamped = self.__tf_buffer.transform_full(point_stamped, target_frame, tf2_ros.Time(), point_stamped.header.frame_id, rclpy.duration.Duration(seconds=.5))

        self.get_logger().debug(f'requested point translated to frame {point_stamped.header.frame_id}: {point_stamped.point}')

        alpha, beta, gamma = self.__inverse_kinematics(point_stamped.point, leg)

        self.__goal_positions[leg.joint_index(Joint.alpha)] = alpha
        self.__goal_positions[leg.joint_index(Joint.beta)]  = beta
        self.__goal_positions[leg.joint_index(Joint.gamma)] = gamma

    def run(self):
        if self.__current_positions is None:
            self.get_logger().warning('not received any joint_states from robot/sim yet')
            return
        
        if self.__current_gait is Gait.T_POSE:
            for leg in Leg:
                self.__goal_positions[leg.joint_index(Joint.alpha)] = pi/4
                self.__goal_positions[leg.joint_index(Joint.beta)]  = pi/4
                self.__goal_positions[leg.joint_index(Joint.gamma)] = pi/4


        elif self.__current_gait is Gait.TRIPOD:
            if self.__gait_step % (self.__gait_granularity*2) < self.__gait_granularity:
                return_stroke = (Leg.LEFT_FRONT, Leg.RIGHT_MIDDLE, Leg.LEFT_REAR)
            else:
                return_stroke = (Leg.RIGHT_FRONT, Leg.LEFT_MIDDLE, Leg.RIGHT_REAR)

            for leg in Leg:

                y = .19
                
                if leg in return_stroke:
                    x = .05 * -cos(pi * (self.__gait_step%self.__gait_granularity) / self.__gait_granularity)
                    z = -.145 + .06 * sin(pi * (self.__gait_step%self.__gait_granularity) / self.__gait_granularity)
                else:
                    x = .05 - 2 * .05 * (self.__gait_step%self.__gait_granularity) / self.__gait_granularity
                    z = -.145

                self.set_leg_goal_coordiante(leg, PointStamped(point=Point(x=x, y=y, z=z), header=Header(frame_id=leg.frame_id(Link.Coxa))))

            self.__gait_step += 1


def main(args=None):
    rclpy.init(args=args)

    akrobat = Akrobat(node_name='akrobat', initial_gait=Gait.T_POSE)

    rclpy.spin(akrobat)

    akrobat.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
