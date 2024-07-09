import rclpy
import rclpy.duration
from rclpy.duration import S_TO_NS
import rclpy.time
from rclpy.node import Node

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from numpy import radians
from math import cos, acos, pi, sin, sqrt, atan2

from builtin_interfaces.msg import Duration
from std_msgs.msg import String, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point
from tf2_geometry_msgs import PointStamped
from sensor_msgs.msg import JointState

from akrobat.enums import *
from akrobat.utils import calculate_angle_between_vectors, rotate_point_around_point


class Akrobat(Node):

    def __init__(self, node_name: str, initial_gait: Gait = Gait.TRIPOD) -> None:
        super().__init__(node_name)

        self.__current_gait = initial_gait
        self.__gait_base_period_seconds = 2
        self.__gait_speed_modifier = 1
        self.joint_names = [f'm{l.value}{j.value+1}'for l in Leg for j in Joint]

        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self)

        self.__current_positions = None
        '''The most up-to-date positions of the robot joints'''

        self.__default_foot_position = Point(x=.0, y=.19, z=-.145)
        '''default position of foot relative to leg origin'''
        self.__default_walk_direction = Point(x=1., y=.0, z=.0)

        self.__gait_toggle = True
        self.__gait_granularity = 3


        self.create_subscription(JointState, 'joint_states', self.__update_positions, 10)
        self.create_subscription(String, '/gait', self.change_gait, 10)
        self.create_subscription(Float64, '/gait_speed', self.change_speed, 10)
        self.__run_timer = self.create_timer(self.gait_period_seconds/2, self.run)

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
        period_sec = self.gait_period_seconds/2
        period_ns = period_sec * S_TO_NS
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

    def coordinate_transform(self, source_point: PointStamped, target_frame: str) -> PointStamped:
        '''
        Calculates the position of the given point in the target_frame's coordinate system.
        '''

        point_stamped = self.__tf_buffer.transform_full(
            object_stamped=point_stamped,
            target_frame=target_frame,
            target_time=tf2_ros.Time(),
            fixed_frame=source_point.header.frame_id,
            timeout=rclpy.duration.Duration(seconds=.5))
        
        return point_stamped

    def change_gait(self, msg: String):
        self.__current_gait = Gait(msg.data)
        self.get_logger().info(f'switched gait to {self.__current_gait}')
        
        # gait toggle is already ready for next gait, so it needs to be reset to update the current gait
        self.__gait_toggle = not self.__gait_toggle
        self.run()

    def change_speed(self, msg: Float64):
        self.gait_speed_modifier = msg.data
        self.get_logger().info(f'changing gait speed to {round(self.gait_speed_modifier*100, 2)}%')

        # gait toggle is already ready for next gait, so it needs to be reset to update the current gait
        self.__gait_toggle = not self.__gait_toggle
        self.run()

    def t_pose(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint(time_from_start=Duration(), positions=[0.]*Leg.amount()*Joint.amount_per_leg())
        trajectory.points = [point]

        self.__joint_trajectory_publisher.publish(trajectory)

    def tripod(self, time_since_gait_start: float = 0, direction_vector: Point = Point(x=1., y=0., z=0.)):
        # TODO add direction parameter so that tripod can walk in x, -x, y and -y


        def calculate_point(time_step: int) -> JointTrajectoryPoint:
            point = JointTrajectoryPoint()

            if self.__gait_toggle:
                return_stroke = (Leg.LEFT_FRONT, Leg.RIGHT_MIDDLE, Leg.LEFT_REAR)
            else:
                return_stroke = (Leg.RIGHT_FRONT, Leg.LEFT_MIDDLE, Leg.RIGHT_REAR)

            positions = [0] * Leg.amount() * Joint.amount_per_leg()

            for leg in Leg:

                y = self.__default_foot_position.y
                is_left_leg = leg.value%2 == 1
                
                if leg in return_stroke:
                    x = .05 * -cos(pi * time_step / self.__gait_granularity)
                    z = self.__default_foot_position.z + .06 * sin(pi * time_step / self.__gait_granularity)
                else:
                    x = .05 - 2 * .05 * time_step / self.__gait_granularity
                    z = self.__default_foot_position.z

                angle = calculate_angle_between_vectors(self.__default_walk_direction, direction_vector)
                _point = rotate_point_around_point(Point(x=x if is_left_leg else -x, y=y, z=z), self.__default_foot_position, angle)

                alpha, beta, gamma = self.__inverse_kinematics(_point)

                positions[leg.joint_index(Joint.alpha)] = alpha
                positions[leg.joint_index(Joint.beta)] = beta
                positions[leg.joint_index(Joint.gamma)] = gamma

            point.positions = positions

            return point
        

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        for time_step in range(self.__gait_granularity-1):
            planned_time_from_start = (time_step/self.__gait_granularity) * (self.gait_period_seconds/2)
            actual_time_from_start = planned_time_from_start - time_since_gait_start

            # only add points in the future
            if actual_time_from_start >= 0:
                point = calculate_point(time_step)
                sec = int(actual_time_from_start)
                nanosec = int(((actual_time_from_start)%1) * S_TO_NS)
                point.time_from_start = Duration(sec=sec, nanosec=nanosec)
                trajectory.points.append(point)

        self.__gait_toggle = not self.__gait_toggle

        # transition to next iteration
        point = calculate_point(0)
        actual_time_from_start = self.gait_period_seconds/2 - time_since_gait_start
        if actual_time_from_start >= 0:
            sec = int(actual_time_from_start)
            nanosec = int(((actual_time_from_start)%1) * S_TO_NS)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            trajectory.points.append(point)

        self.__joint_trajectory_publisher.publish(trajectory)

    def wave(self, time_since_gait_start: float = 0):
        raise NotImplementedError
    
    def ripple(self, time_since_gait_start: float = 0):
        raise NotImplementedError

    def run(self):
        offset = self.__run_timer.time_since_last_call() / S_TO_NS
        if self.__current_gait is Gait.T_POSE:
            self.t_pose()
        elif self.__current_gait is Gait.TRIPOD:
            self.tripod(offset, Point(x=1., y=1., z=0.))
        elif self.__current_gait is Gait.WAVE:
            self.wave(offset)
        elif self.__current_gait is Gait.RIPPLE:
            self.ripple(offset)
        
    def lay_down(self):
        
        stand_trajectory_point = JointTrajectoryPoint()
        stand_trajectory_point.time_from_start = Duration(nanosec = int(.75 * S_TO_NS))

        lay_trajectory_point = JointTrajectoryPoint()
        lay_trajectory_point.time_from_start = Duration(nanosec = int(1.5 * S_TO_NS))

        stretch_trajectory_point = JointTrajectoryPoint()
        stretch_trajectory_point.time_from_start = Duration(nanosec = int(2.25 * S_TO_NS))
        stretch_trajectory_point.positions = [0.] * Leg.amount() * Joint.amount_per_leg()

        for leg in Leg:
            stand_point = Point(x=0., y=self.__default_foot_position.y, z=self.__default_foot_position.z)
            stand_trajectory_point.positions.extend(self.__inverse_kinematics(stand_point))

            lay_point = Point(x=0., y=self.__default_foot_position.y, z=0.)
            lay_trajectory_point.positions.extend(self.__inverse_kinematics(lay_point))

        trajectory = JointTrajectory()
        trajectory.points = [stand_trajectory_point, lay_trajectory_point, stretch_trajectory_point]
        trajectory.joint_names = self.joint_names

        self.__joint_trajectory_publisher.publish(trajectory)

    def stand_up(self):
        
        stretch_trajectory_point = JointTrajectoryPoint()
        stretch_trajectory_point.time_from_start = Duration(nanosec = int(.75 * S_TO_NS))
        stretch_trajectory_point.positions = [0.] * Leg.amount() * Joint.amount_per_leg()

        lay_trajectory_point = JointTrajectoryPoint()
        lay_trajectory_point.time_from_start = Duration(nanosec = int(1.5 * S_TO_NS))

        stand_trajectory_point = JointTrajectoryPoint()
        stand_trajectory_point.time_from_start = Duration(nanosec = int(2.25 * S_TO_NS))

        for leg in Leg:
            lay_point = Point(x=0., y=self.__default_foot_position.y, z=0.)
            lay_trajectory_point.positions.extend(self.__inverse_kinematics(lay_point))

            stand_point = Point(x=0., y=self.__default_foot_position.y, z=self.__default_foot_position.z)
            stand_trajectory_point.positions.extend(self.__inverse_kinematics(stand_point))

        trajectory = JointTrajectory()
        trajectory.points = [stand_trajectory_point, lay_trajectory_point, stretch_trajectory_point]
        trajectory.joint_names = self.joint_names

        self.__joint_trajectory_publisher.publish(trajectory)

    def shutdown(self):
        self.__run_timer.cancel()
        self.lay_down()


def main(args=None):
    rclpy.init(args=args)

    akrobat = Akrobat(node_name='akrobat', initial_gait=Gait.TRIPOD)
    try:
        rclpy.spin(akrobat)
    finally:
        akrobat.shutdown()
        akrobat.destroy_node()
        rclpy.shutdown()