from collections import namedtuple
from enum import Enum

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.action import ActionClient


from numpy import radians
from math import cos, acos, pi, sin, sqrt, degrees, atan2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros

from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray, Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point, Transform
from tf2_geometry_msgs import PointStamped
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory

class Gait(Enum):
    T_POSE  = 0
    TRIPOD  = 1
    WAVE    = 2
    RIPPLE  = 3


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
        self.__gait_base_period_seconds = 4
        self.__gait_speed_modifier = 1
        self.__robot_frame_name = 'akrobat_link' # TODO get from URDF?
        self.joint_names = [f'm{l.value}{j.value+1}'for l in Leg for j in Joint]

        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self)

        self.__current_positions = None
        '''The most up-to-date positions of the robot joints'''
        self.__goal_positions = [0.] * 3 * Leg.amount()
        '''The desired positions of the robot joints'''

        self.__gait_toggle = True
        self.__gait_granularity = 3

        self.create_subscription(JointState, 'joint_states', self.__update_positions, 10)
        self.__run_timer = self.create_timer(self.__gait_base_period_seconds/2, self.tripod)
        self.create_subscription(PointStamped, 'goal_point', self.__set_goal_point, 10)

        self.__joint_trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

    @property
    def gait_period_seconds(self):
        return self.__gait_base_period_seconds / self.gait_speed_modifier

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
        period_sec = self.__gait_period_time / self.__gait_speed_modifier
        period_ns = period_sec / (1000**3)
        self.__run_timer.timer_period_ns = period_ns

    def __update_positions(self, joint_state: JointState):
        self.__current_positions = joint_state.position

    def __inverse_kinematics(self, point: Point) -> tuple[float]:
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

            return -alpha, -beta, -gamma
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

    def tripod(self):
        # TODO add direction parameter so that tripod can walk in x, -x, y and -y


        def calculate_point(time_step: int) -> JointTrajectoryPoint:
            point = JointTrajectoryPoint()

            if self.__gait_toggle:
                return_stroke = (Leg.LEFT_FRONT, Leg.RIGHT_MIDDLE, Leg.LEFT_REAR)
            else:
                return_stroke = (Leg.RIGHT_FRONT, Leg.LEFT_MIDDLE, Leg.RIGHT_REAR)

            positions = [0] * Leg.amount() * Joint.amount_per_leg()

            for leg in Leg:

                y = .19
                
                if leg in return_stroke:
                    x = .05 * -cos(pi * time_step / self.__gait_granularity)
                    z = -.145 + .06 * sin(pi * time_step / self.__gait_granularity)
                else:
                    x = .05 - 2 * .05 * time_step / self.__gait_granularity
                    z = -.145

                alpha, beta, gamma = self.__inverse_kinematics(Point(x=x, y=y, z=z))

                positions[leg.joint_index(Joint.alpha)] = alpha
                positions[leg.joint_index(Joint.beta)] = beta
                positions[leg.joint_index(Joint.gamma)] = gamma

            point.positions = positions

            return point
        

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        for time_step in range(self.__gait_granularity-1):
            point = calculate_point(time_step)
            point.time_from_start = Duration(nanosec = int((time_step/self.__gait_granularity) * (self.gait_period_seconds/2 * 1000**3)))
            trajectory.points.append(point)

        self.__gait_toggle = not self.__gait_toggle

        # transition to next iteration
        point = calculate_point(0)
        point.time_from_start = Duration(nanosec= int(self.gait_period_seconds/2 * 1000**3))
        trajectory.points.append(point)

        self.__joint_trajectory_publisher.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)

    akrobat = Akrobat(node_name='akrobat', initial_gait=Gait.TRIPOD)

    rclpy.spin(akrobat)

    akrobat.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
