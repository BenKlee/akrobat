import numpy as np
from numpy import radians
from math import cos, acos, pi, sin, sqrt, degrees, atan2

from akrobat import enums
from akrobat.msg import movement
from akrobat.trajectory import Trajectory
from akrobat.rumble_pad_2 import RumblePad2
from akrobat.leg_settings import LegSettings
from akrobat.coordinate_system import CoordinateSystem
from akrobat.helper import from_homogenous, to_homogenous
from akrobat.trajectory_settings import TrajectorySettings

import rospy
from tf import TransformerROS
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler

X = 0
Y = 1
Z = 2


class Akrobat:


    def __init__(self) -> None:
        self.gait = enums.Gait.INIT
        self.body_rotation = 0
        self.rolled_over = False

        self.length_coxa = 72
        self.length_femur = 92
        self.length_tibia = 162
        self.scale_factor_translation = 50
        self.scale_factor_rotation = 10

        self.leg_settings = [None] * enums.Leg.amount()
        self.__init_leg_settings()

        self.trajectory_data = Trajectory()
        self.trajectory_settings = [0] * 3
        self.trajectory_settings[enums.Gait.TRIPOD] = TrajectorySettings(40, 40, 15, 2)
        self.trajectory_settings[enums.Gait.WAVE]   = TrajectorySettings(40, 40, 15, 6)
        self.trajectory_settings[enums.Gait.RIPPLE] = TrajectorySettings(40, 40, 15, 6)


        self.joint_publisher = rospy.Publisher('/goal_joint_states', JointState, queue_size=1)
        self.movement_subscriber = rospy.Subscriber('movements', movement, self.rumble_pad_2_callback, queue_size=5)
        

        self.leg_coordinate_system = CoordinateSystem()
        self.body_coordinate_system = CoordinateSystem()
        self.main_coordinate_system = CoordinateSystem()
        self.foot_coordinate_system = CoordinateSystem()
        self.joint_state = JointState(
            name = ['m11','m12','m13','m21','m22','m23','m31','m32','m33','m41','m42','m43','m51','m52','m53','m61','m62','m63'],
            position = [0] * enums.Leg.amount() * 3,
            velocity = [0] * enums.Leg.amount() * 3,
            effort   = [0] * enums.Leg.amount() * 3
        )

        self.pad = RumblePad2()
        

    def __init_leg_settings(self):
            self.leg_settings[enums.Leg.LEFT_FRONT] = LegSettings(0.0, -160.0, -51.0, 217.0, 0.0, 160.0, 10.0, -90.0, -26.0, -99.0, -135.0, 65.0, 96.0, 135.0)
            self.leg_settings[enums.Leg.RIGHT_FRONT] = LegSettings(0.0, -20.0, 51.0, 217.0, 0.0, 20.0, 10.0, -90.0, -71.0, -99.0, -135.0, 28.0, 96.0, 135.0)
            self.leg_settings[enums.Leg.LEFT_MIDDLE] = LegSettings(0.0, 180.0, -51.0, 0.0, 0.0, 180.0, 10.0, -90.0, -51.0, -99.0, -135.0, 48.0, 96.0, 135.0)
            self.leg_settings[enums.Leg.RIGHT_MIDDLE] = LegSettings(0.0, 0.0, 51.0, 0.0, 0.0, 0.0, 10.0, -90.0, -51.0, -99.0, -135.0, 48.0, 96.0, 135.0)
            self.leg_settings[enums.Leg.LEFT_REAR] = LegSettings(0.0, 160.0, -51.0, -217.0, 0.0, -160.0, 10.0, -90.0, -71.0, -99.0, -135.0, 30.0, 96.0, 135.0)
            self.leg_settings[enums.Leg.RIGHT_REAR] = LegSettings(0.0, 20.0, 51.0, -217.0, 0.0, -20.0, 10.0, -90.0, -23.0, -107.0, -135.0, 75.0, 96.0, 135.0)


    def init_akrobat(self):

        tf = TransformerROS()

        for leg in enums.Leg:
            # leg coordinate system
            t = tf.fromTranslationRotation((self.length_tibia,0,0), quaternion_from_euler(*radians((0,0,0))))
            new_position = t.dot(to_homogenous(self.leg_coordinate_system.leg[leg].foot_initial_position))
            self.leg_coordinate_system.leg[leg].foot_initial_position = from_homogenous(new_position)

            t = tf.fromTranslationRotation((0,0,0), quaternion_from_euler(*radians((0,0, -self.leg_settings[leg].joint_initial_gamma))))
            new_position = t.dot(to_homogenous(self.leg_coordinate_system.leg[leg].foot_initial_position))
            self.leg_coordinate_system.leg[leg].foot_initial_position = from_homogenous(new_position)

            t = tf.fromTranslationRotation((self.length_femur, 0, 0), quaternion_from_euler(*radians((0,0,0))))
            new_position = t.dot(to_homogenous(self.leg_coordinate_system.leg[leg].foot_initial_position))
            self.leg_coordinate_system.leg[leg].foot_initial_position = from_homogenous(new_position)

            t = tf.fromTranslationRotation((0,0,0), quaternion_from_euler(*radians((0,0,-self.leg_settings[leg].joint_initial_beta))))
            new_position = t.dot(to_homogenous(self.leg_coordinate_system.leg[leg].foot_initial_position))
            self.leg_coordinate_system.leg[leg].foot_initial_position = from_homogenous(new_position)

            t = tf.fromTranslationRotation((self.length_coxa, 0, 0), quaternion_from_euler(*radians((0,0,0))))
            new_position = t.dot(to_homogenous(self.leg_coordinate_system.leg[leg].foot_initial_position))
            self.leg_coordinate_system.leg[leg].foot_initial_position = from_homogenous(new_position)

            t = tf.fromTranslationRotation((0,0,0), quaternion_from_euler(*radians((-90, 0, self.leg_settings[leg].joint_initial_alpha))))
            new_position = t.dot(to_homogenous(self.leg_coordinate_system.leg[leg].foot_initial_position))
            self.leg_coordinate_system.leg[leg].foot_initial_position = from_homogenous(new_position)


            # body coordinate system
            t = tf.fromTranslationRotation((self.leg_settings[leg].body_const_x, self.leg_settings[leg].body_const_y, self.leg_settings[leg].body_const_z), quaternion_from_euler(*radians((0,0,0))))
            new_position = t.dot(to_homogenous(self.leg_coordinate_system.leg[leg].foot_initial_position))
            self.body_coordinate_system.leg[leg].foot_global_position = from_homogenous(new_position)


            # main coordinate system
            t = tf.fromTranslationRotation((0, 0, 0), quaternion_from_euler(*radians((0, 0, 0))))
            new_position = t.dot(to_homogenous(self.body_coordinate_system.leg[leg].foot_global_position))
            self.main_coordinate_system.leg[leg].foot_global_position = from_homogenous(new_position)


            # rviz init position
            self.joint_state.header.stamp = rospy.Time.now()
            self.coordinate_transformation(leg)

            self.inverse_kinematics(
                self.leg_coordinate_system.leg[leg].foot_present_position[X],
                self.leg_coordinate_system.leg[leg].foot_present_position[Y],
                self.leg_coordinate_system.leg[leg].foot_present_position[Z],
                leg
            )
            self.move_leg(
                self.leg_coordinate_system.leg[leg].joint_angles.alpha,
                self.leg_coordinate_system.leg[leg].joint_angles.beta,
                self.leg_coordinate_system.leg[leg].joint_angles.gamma,
                leg
            )

            self.joint_publisher.publish(self.joint_state)


    def coordinate_transformation(self, leg: enums.Leg):
        
        tf = TransformerROS()
        ls = self.leg_settings[leg]


        t = tf.fromTranslationRotation(self.leg_coordinate_system.leg[leg].foot_initial_position, quaternion_from_euler(*radians((0,0,0))))
        new_position = from_homogenous(t.dot(to_homogenous(self.foot_coordinate_system.leg[leg].trajectory_present_position)))
        self.leg_coordinate_system.leg[leg].foot_present_position = new_position


        t = tf.fromTranslationRotation((ls.body_const_x, ls.body_const_y, ls.body_const_z), quaternion_from_euler(*radians((0,0,0))))
        new_position = from_homogenous(t.dot(to_homogenous(self.leg_coordinate_system.leg[leg].foot_present_position)))
        self.body_coordinate_system.leg[leg].foot_global_position = new_position


        t = tf.fromTranslationRotation((0,0,0), quaternion_from_euler(*radians((0,0,0))))
        new_position = from_homogenous(t.dot(to_homogenous(self.body_coordinate_system.leg[leg].foot_global_position)))
        self.main_coordinate_system.leg[leg].foot_global_position = new_position


        pad_translation = self.pad.body_translation
        pad_rotation = self.pad.body_rotation
        t = tf.fromTranslationRotation((pad_translation[X], pad_translation[Y] + ls.roll_over, pad_translation[Z]), quaternion_from_euler(*radians((pad_rotation[X] + self.body_rotation, pad_rotation[Y], pad_rotation[Z]))))
        t = np.linalg.inv(t)
        new_position = from_homogenous(t.dot(to_homogenous(self.main_coordinate_system.leg[leg].foot_global_position)))
        self.body_coordinate_system.leg[leg].foot_global_position = new_position


        t = tf.fromTranslationRotation((ls.body_const_x, ls.body_const_y, ls.body_const_z), quaternion_from_euler(*radians((0,0,0))))
        t = np.linalg.inv(t)
        new_position = from_homogenous(t.dot(to_homogenous(self.body_coordinate_system.leg[leg].foot_global_position)))
        self.leg_coordinate_system.leg[leg].foot_present_position = new_position

        t = tf.fromTranslationRotation((0,0,0), quaternion_from_euler(*radians((0,0, ls.rotation_of_coxa))))
        new_position = from_homogenous(t.dot(to_homogenous(self.leg_coordinate_system.leg[leg].foot_present_position)))
        self.leg_coordinate_system.leg[leg].foot_present_position = new_position


    def inverse_kinematics(self, x: float, y: float, z: float, leg: enums.Leg):
        '''
        Calculates the inverse kinematics for a specific leg

                Parameters:
                        x   (float):    the x coordinate for the current position of the leg
                        y   (float):    the y coordinate for the current position of the leg
                        z   (float):    the z coordinate for the current position of the leg
                        leg (Leg):      the leg to execute this operation on
        '''

        # FUNKTIONIERT WIE IN C++

        R = sqrt(y**2 + x**2)
        ALPHA = atan2(y, x)
        L = sqrt(((R - self.length_coxa)**2) + z**2)
        BETA1 = atan2(z, (R - self.length_coxa))
        BETA2 = acos(0.5 * ((self.length_femur**2) + (L**2) - (self.length_tibia**2)) / (L * self.length_femur));

        GAMMA = acos(0.5 * ((self.length_tibia**2) + (self.length_femur**2) - (L**2)) / (self.length_femur * self.length_tibia));

        if not self.rolled_over:
            BETA = BETA1 + BETA2
            GAMMA = GAMMA - radians(180)
        else:
            BETA = BETA1 - BETA2
            GAMMA = radians(180) - GAMMA

        self.leg_coordinate_system.leg[leg].joint_angles.alpha = degrees(ALPHA)
        self.leg_coordinate_system.leg[leg].joint_angles.beta  = degrees(BETA)
        self.leg_coordinate_system.leg[leg].joint_angles.gamma = degrees(GAMMA)

        
    def move_leg(self, alpha: float, beta: float, gamma: float, leg: enums.Leg) -> bool:
        '''
        Move leg to target position
        
                Parameters:
                        alpha   (float):    the angle for coxa
                        beta    (float):    the angle for femur
                        gamma   (float):    the angle for tibia
                        leg     (Leg):      the leg to execute this operation on

                Returns:
                        success (bool):     wether or not this operation can be executed because of the angle limits
        '''

        if not self.within_limits(alpha, self.leg_settings[leg].minimum_alpha, self.leg_settings[leg].maximum_alpha):
            print(f'[WARNING] LEG {leg}: angle range of coxa/alpha ({self.leg_settings[leg].minimum_alpha}:{self.leg_settings[leg].maximum_alpha}) joint is exceeded {alpha}')
            return False

        if not self.within_limits(beta, self.leg_settings[leg].minimum_beta, self.leg_settings[leg].maximum_beta):
            print(f'[WARNING] LEG {leg}: angle range of femur/beta ({self.leg_settings[leg].minimum_beta}:{self.leg_settings[leg].maximum_beta}) joint is exceeded {beta}')
            return False

        if not self.within_limits(gamma, self.leg_settings[leg].minimum_gamma, self.leg_settings[leg].maximum_gamma):
            print(f'[WARNING] LEG {leg}: angle range of tibia/gamma ({self.leg_settings[leg].minimum_gamma}:{self.leg_settings[leg].maximum_gamma}) joint is exceeded {gamma}')
            return False

        self.joint_state.position[leg.joint_index(enums.Joint.alpha)] = radians(alpha)
        self.joint_state.position[leg.joint_index(enums.Joint.beta)]  = radians(beta)
        self.joint_state.position[leg.joint_index(enums.Joint.gamma)] = radians(gamma)

        return True


    def within_limits(self, value: float, min: float, max: float) -> bool:
        '''
        Checks wether a value is within the given limits

                Parameters:
                        value (float):  the value to check
                        min (float):    the minimum the value should be
                        max (float):    the maximum the value should be
        '''
        return value >= min and value <= max

    @property
    def is_moving(self) -> bool:
        for i in range(3):
            if abs(self.pad.speed[i]) > .3:
                return True

        return False


    @property
    def is_translating(self) -> bool:
        for i in range(3):
            if abs(self.pad.body_translation[i]) > .3:
                return True

        return False


    @property
    def is_rotating(self) -> bool:
        for i in range(3):
            if abs(self.pad.body_rotation[i]) > .3:
                return True

        return False


    def rumble_pad_2_callback(self, movement: movement) -> None:

        if movement.macro == 'shutdown':
            print('SHUTTING DOWN!')
            rospy.signal_shutdown()
        else:
            if movement.macro == 'start':
                self.joint_state.header.stamp = rospy.Time.now()

                if not self.rolled_over:
                    self.rolled_over = True
                    self.body_rotation = 180

                    self.leg_settings[enums.Leg.LEFT_FRONT].rotation_of_coxa = 160
                    self.leg_settings[enums.Leg.RIGHT_FRONT].rotation_of_coxa = 20
                    self.leg_settings[enums.Leg.LEFT_MIDDLE].rotation_of_coxa = 180
                    self.leg_settings[enums.Leg.RIGHT_MIDDLE].rotation_of_coxa = 0
                    self.leg_settings[enums.Leg.LEFT_REAR].rotation_of_coxa = -160
                    self.leg_settings[enums.Leg.RIGHT_REAR].rotation_of_coxa = -20

                    self.leg_settings[enums.Leg.LEFT_FRONT].roll_over = 2 * self.leg_settings[enums.Leg.LEFT_FRONT].body_const_y
                    self.leg_settings[enums.Leg.RIGHT_FRONT].roll_over = 2 * self.leg_settings[enums.Leg.RIGHT_FRONT].body_const_y
                    self.leg_settings[enums.Leg.LEFT_MIDDLE].roll_over = 0
                    self.leg_settings[enums.Leg.RIGHT_MIDDLE].roll_over = 0
                    self.leg_settings[enums.Leg.LEFT_REAR].roll_over = 2 * self.leg_settings[enums.Leg.LEFT_REAR].body_const_y
                    self.leg_settings[enums.Leg.RIGHT_REAR].roll_over = 2 * self.leg_settings[enums.Leg.RIGHT_REAR].body_const_y

                    for leg in enums.Leg:
                        self.coordinate_transformation(leg)
                        x = self.leg_coordinate_system.leg[leg].foot_pres_pos[X]
                        y = self.leg_coordinate_system.leg[leg].foot_pres_pos[Y]
                        z = self.leg_coordinate_system.leg[leg].foot_pres_pos[Z]
                        self.inverse_kinematics(x, y, z, leg)
                        alpha = self.leg_coordinate_system.leg[leg].jointAngles.alpha
                        beta = self.leg_coordinate_system.leg[leg].jointAngles.beta
                        gamma = self.leg_coordinate_system.leg[leg].jointAngles.gamma
                        self.move_leg(alpha, beta, gamma, leg)

                elif self.rolled_over:
                    self.rolled_over = False
                    self.body_rotation = 0

                    self.leg_settings[enums.Leg.LEFT_FRONT].rotation_of_coxa = -160
                    self.leg_settings[enums.Leg.RIGHT_FRONT].rotation_of_coxa = -20
                    self.leg_settings[enums.Leg.LEFT_MIDDLE].rotation_of_coxa = 180
                    self.leg_settings[enums.Leg.RIGHT_MIDDLE].rotation_of_coxa = 0
                    self.leg_settings[enums.Leg.LEFT_REAR].rotation_of_coxa = 160
                    self.leg_settings[enums.Leg.RIGHT_REAR].rotation_of_coxa = 20

                    self.leg_settings[enums.Leg.LEFT_FRONT].roll_over = 0
                    self.leg_settings[enums.Leg.RIGHT_FRONT].roll_over = 0
                    self.leg_settings[enums.Leg.LEFT_MIDDLE].roll_over = 0
                    self.leg_settings[enums.Leg.RIGHT_MIDDLE].roll_over = 0
                    self.leg_settings[enums.Leg.LEFT_REAR].roll_over = 0
                    self.leg_settings[enums.Leg.RIGHT_REAR].roll_over = 0

                    for leg in enums.Leg:
                        self.coordinate_transformation(leg)
                        x = self.leg_coordinate_system.leg[leg].foot_pres_pos[X]
                        y = self.leg_coordinate_system.leg[leg].foot_pres_pos[Y]
                        z = self.leg_coordinate_system.leg[leg].foot_pres_pos[Z]
                        self.inverse_kinematics(x, y, z, leg)
                        alpha = self.leg_coordinate_system.leg[leg].jointAngles.alpha
                        beta = self.leg_coordinate_system.leg[leg].jointAngles.beta
                        gamma = self.leg_coordinate_system.leg[leg].jointAngles.gamma
                        self.move_leg(alpha, beta, gamma, leg)

                self.joint_publisher.publish(self.joint_state)

            elif movement.walking_mode != self.gait.name.lower():
                self.gait = enums.Gait[movement.walking_mode.upper()]

                if self.gait is enums.Gait.TRIPOD:
                    self.trajectory_data.case_step = [1,0,0,1,1,0]

                elif self.gait is enums.Gait.WAVE:
                    self.trajectory_data.case_step = [0,3,1,4,2,5]

                elif self.gait is enums.Gait.RIPPLE:
                    self.trajectory_data.case_step = [4,1,2,5,0,3]

                if self.gait is not enums.Gait.RESET:
                    self.trajectory_data.initial_amplitude_x = self.trajectory_settings[self.gait].amplitude_width
                    self.trajectory_data.initial_amplitude_y = self.trajectory_settings[self.gait].amplitude_width
                    self.trajectory_data.initial_amplitude_z = self.trajectory_settings[self.gait].amplitude_height
                else:
                    self.move_leg(-20, 10, -90, enums.Leg.LEFT_FRONT)
                    self.move_leg( 20, 10, -90, enums.Leg.RIGHT_FRONT)
                    self.move_leg(  0, 10, -90, enums.Leg.LEFT_MIDDLE)
                    self.move_leg(  0, 10, -90, enums.Leg.RIGHT_MIDDLE)
                    self.move_leg( 20, 10, -90, enums.Leg.LEFT_REAR)
                    self.move_leg(-20, 10, -90, enums.Leg.RIGHT_REAR)

            
            # translation
            self.pad.body_translation[X] = (movement.commands[7] * self.scale_factor_translation) / 32767
            self.pad.body_translation[Y] = (movement.commands[6] * self.scale_factor_translation) / 32767
            self.pad.body_translation[Z] = (movement.commands[8] * self.scale_factor_translation) / 32767


            # walking
            if movement.commands[1] != 0:
                command = movement.commands[1]
                self.pad.speed[X] = -(command / abs(command))
            else:
                self.pad.speed[X] = 0

            
            if movement.commands[0] != 0:
                command = movement.commands[0]
                self.pad.speed[Y] = -(command / abs(command))
            else:
                self.pad.speed[Y] = 0
                

            if movement.commands[2] != 0:
                command = movement.commands[2]
                self.pad.speed[Z] = -(command / abs(command))
            else:
                self.pad.speed[Z] = 0


            # rotation
            self.pad.body_rotation[X] = (movement.commands[4] * self.scale_factor_rotation) / 32767
            self.pad.body_rotation[Y] = (movement.commands[5] * self.scale_factor_rotation) / 32767
            self.pad.body_rotation[Z] = (movement.commands[3] * self.scale_factor_rotation) / 32767


            speed = self.pad.speed

            if abs(speed[X] < abs(speed[Y])):
                # leg 1 amplitude                                                                   # leg 2 amplitude
                self.trajectory_data.amplitude_x[enums.Leg.LEFT_FRONT]  = self.trajectory_data.initial_amplitude_x * speed[X]
                self.trajectory_data.amplitude_x[enums.Leg.RIGHT_FRONT] = self.trajectory_data.initial_amplitude_x * speed[X]
                self.trajectory_data.amplitude_y[enums.Leg.LEFT_FRONT]  = self.trajectory_data.initial_amplitude_y * speed[Y]
                self.trajectory_data.amplitude_y[enums.Leg.RIGHT_FRONT] = self.trajectory_data.initial_amplitude_y * speed[Y]
                self.trajectory_data.amplitude_z[enums.Leg.LEFT_FRONT]  = self.trajectory_data.initial_amplitude_z * speed[Y]
                self.trajectory_data.amplitude_z[enums.Leg.RIGHT_FRONT] = self.trajectory_data.initial_amplitude_z * speed[Y]


                # leg 3 amplitude                                                                   # leg 4 amplitude
                self.trajectory_data.amplitude_x[enums.Leg.LEFT_MIDDLE]  = self.trajectory_data.initial_amplitude_x * speed[X]
                self.trajectory_data.amplitude_x[enums.Leg.RIGHT_MIDDLE] = self.trajectory_data.initial_amplitude_x * speed[X]
                self.trajectory_data.amplitude_y[enums.Leg.LEFT_MIDDLE]  = self.trajectory_data.initial_amplitude_y * speed[Y]
                self.trajectory_data.amplitude_y[enums.Leg.RIGHT_MIDDLE] = self.trajectory_data.initial_amplitude_y * speed[Y]
                self.trajectory_data.amplitude_z[enums.Leg.LEFT_MIDDLE]  = self.trajectory_data.initial_amplitude_z * speed[Y]
                self.trajectory_data.amplitude_z[enums.Leg.RIGHT_MIDDLE] = self.trajectory_data.initial_amplitude_z * speed[Y]


                # leg 5 amplitude                                                                   # leg 6 amplitude
                self.trajectory_data.amplitude_x[enums.Leg.LEFT_REAR]  = self.trajectory_data.initial_amplitude_x * speed[X]
                self.trajectory_data.amplitude_x[enums.Leg.RIGHT_REAR] = self.trajectory_data.initial_amplitude_x * speed[X]
                self.trajectory_data.amplitude_y[enums.Leg.LEFT_REAR]  = self.trajectory_data.initial_amplitude_y * speed[Y]
                self.trajectory_data.amplitude_y[enums.Leg.RIGHT_REAR] = self.trajectory_data.initial_amplitude_y * speed[Y]
                self.trajectory_data.amplitude_z[enums.Leg.LEFT_REAR]  = self.trajectory_data.initial_amplitude_z * speed[Y]
                self.trajectory_data.amplitude_z[enums.Leg.RIGHT_REAR] = self.trajectory_data.initial_amplitude_z * speed[Y]


            elif abs(speed[X]) > abs(speed[Y]):
                # leg 1 amplitude                                                                   # leg 2 amplitude
                self.trajectory_data.amplitude_x[enums.Leg.LEFT_FRONT]  = self.trajectory_data.initial_amplitude_x * speed[X]
                self.trajectory_data.amplitude_x[enums.Leg.RIGHT_FRONT] = self.trajectory_data.initial_amplitude_x * speed[X]
                self.trajectory_data.amplitude_y[enums.Leg.LEFT_FRONT]  = self.trajectory_data.initial_amplitude_y * speed[Y]
                self.trajectory_data.amplitude_y[enums.Leg.RIGHT_FRONT] = self.trajectory_data.initial_amplitude_y * speed[Y]
                self.trajectory_data.amplitude_z[enums.Leg.LEFT_FRONT]  = self.trajectory_data.initial_amplitude_z * speed[X]
                self.trajectory_data.amplitude_z[enums.Leg.RIGHT_FRONT] = self.trajectory_data.initial_amplitude_z * speed[X]


                # leg 3 amplitude                                                                   # leg 4 amplitude
                self.trajectory_data.amplitude_x[enums.Leg.LEFT_MIDDLE]  = self.trajectory_data.initial_amplitude_x * speed[X]
                self.trajectory_data.amplitude_x[enums.Leg.RIGHT_MIDDLE] = self.trajectory_data.initial_amplitude_x * speed[X]
                self.trajectory_data.amplitude_y[enums.Leg.LEFT_MIDDLE]  = self.trajectory_data.initial_amplitude_y * speed[Y]
                self.trajectory_data.amplitude_y[enums.Leg.RIGHT_MIDDLE] = self.trajectory_data.initial_amplitude_y * speed[Y]
                self.trajectory_data.amplitude_z[enums.Leg.LEFT_MIDDLE]  = self.trajectory_data.initial_amplitude_z * speed[X]
                self.trajectory_data.amplitude_z[enums.Leg.RIGHT_MIDDLE] = self.trajectory_data.initial_amplitude_z * speed[X]


                # leg 5 amplitude                                                                   # leg 6 amplitude
                self.trajectory_data.amplitude_x[enums.Leg.LEFT_REAR]  = self.trajectory_data.initial_amplitude_x * speed[X]
                self.trajectory_data.amplitude_x[enums.Leg.RIGHT_REAR] = self.trajectory_data.initial_amplitude_x * speed[X]
                self.trajectory_data.amplitude_y[enums.Leg.LEFT_REAR]  = self.trajectory_data.initial_amplitude_y * speed[Y]
                self.trajectory_data.amplitude_y[enums.Leg.RIGHT_REAR] = self.trajectory_data.initial_amplitude_y * speed[Y]
                self.trajectory_data.amplitude_z[enums.Leg.LEFT_REAR]  = self.trajectory_data.initial_amplitude_z * speed[X]
                self.trajectory_data.amplitude_z[enums.Leg.RIGHT_REAR] = self.trajectory_data.initial_amplitude_z * speed[X]
            elif speed[X] == 0 and speed[Y] == 0 and speed[Z] != 0:

                # leg 1 amplitude                                                                   # leg 2 amplitude
                self.trajectory_data.amplitude_x[enums.Leg.LEFT_FRONT]  = -self.trajectory_data.initial_amplitude_x * speed[Z]
                self.trajectory_data.amplitude_x[enums.Leg.RIGHT_FRONT] = -self.trajectory_data.initial_amplitude_x * speed[Z]
                self.trajectory_data.amplitude_y[enums.Leg.LEFT_FRONT]  = -self.trajectory_data.initial_amplitude_y * speed[Z]
                self.trajectory_data.amplitude_y[enums.Leg.RIGHT_FRONT] =  self.trajectory_data.initial_amplitude_y * speed[Z]
                self.trajectory_data.amplitude_z[enums.Leg.LEFT_FRONT]  =  self.trajectory_data.initial_amplitude_z * speed[Z]
                self.trajectory_data.amplitude_z[enums.Leg.RIGHT_FRONT] =  self.trajectory_data.initial_amplitude_z * speed[Z]


                # leg 3 amplitude                                                                   # leg 4 amplitude
                self.trajectory_data.amplitude_x[enums.Leg.LEFT_MIDDLE]  =  0
                self.trajectory_data.amplitude_x[enums.Leg.RIGHT_MIDDLE] =  0
                self.trajectory_data.amplitude_y[enums.Leg.LEFT_MIDDLE]  = -self.trajectory_data.initial_amplitude_y * speed[Z]
                self.trajectory_data.amplitude_y[enums.Leg.RIGHT_MIDDLE] =  self.trajectory_data.initial_amplitude_y * speed[Z]
                self.trajectory_data.amplitude_z[enums.Leg.LEFT_MIDDLE]  =  self.trajectory_data.initial_amplitude_z * speed[Z]
                self.trajectory_data.amplitude_z[enums.Leg.RIGHT_MIDDLE] =  self.trajectory_data.initial_amplitude_z * speed[Z]


                # leg 5 amplitude                                                                   # leg 6 amplitude
                self.trajectory_data.amplitude_x[enums.Leg.LEFT_REAR]  =  self.trajectory_data.initial_amplitude_x * speed[Z]
                self.trajectory_data.amplitude_x[enums.Leg.RIGHT_REAR] =  self.trajectory_data.initial_amplitude_x * speed[Z]
                self.trajectory_data.amplitude_y[enums.Leg.LEFT_REAR]  = -self.trajectory_data.initial_amplitude_y * speed[Z]
                self.trajectory_data.amplitude_y[enums.Leg.RIGHT_REAR] =  self.trajectory_data.initial_amplitude_y * speed[Z]
                self.trajectory_data.amplitude_z[enums.Leg.LEFT_REAR]  =  self.trajectory_data.initial_amplitude_z * speed[Z]
                self.trajectory_data.amplitude_z[enums.Leg.RIGHT_REAR] =  self.trajectory_data.initial_amplitude_z * speed[Z]


    def run_akrobat(self):


        if self.is_moving or self.is_translating or self.is_rotating:

            self.joint_state.header.stamp = rospy.Time.now()


            for leg in enums.Leg:
                
                if self.gait is enums.Gait.TRIPOD:
                    self.tripod_gait(leg)
                
                if self.gait is enums.Gait.WAVE:
                    self.wave_gait(leg)

                if self.gait is enums.Gait.RIPPLE:
                    self.ripple_gate(leg)



                self.coordinate_transformation(leg)
                foot_present_position = self.leg_coordinate_system.leg[leg].foot_present_position
                joint_angles = self.leg_coordinate_system.leg[leg].joint_angles
                self.inverse_kinematics(foot_present_position[X], foot_present_position[Y], foot_present_position[Z], leg)
                self.move_leg(joint_angles.alpha, joint_angles.beta, joint_angles.gamma, leg)
            
            
            self.trajectory_data.tick = (self.trajectory_data.tick + 1) % self.trajectory_settings[self.gait].max_ticks

            # new loop, move every leg one case step forward
            if self.trajectory_data.tick == 0:
                for leg in enums.Leg:
                    # modulo 1 ist immer 0
                    self.trajectory_data.case_step[leg] = (self.trajectory_data.case_step[leg] + 1) % self.trajectory_settings[self.gait].number_of_case_steps


        else:
            self.move_leg(-20, 10, -90, enums.Leg.LEFT_FRONT)
            self.move_leg( 20, 10, -90, enums.Leg.RIGHT_FRONT)
            self.move_leg(  0, 10, -90, enums.Leg.LEFT_MIDDLE)
            self.move_leg(  0, 10, -90, enums.Leg.RIGHT_MIDDLE)
            self.move_leg( 20, 10, -90, enums.Leg.LEFT_REAR)
            self.move_leg(-20, 10, -90, enums.Leg.RIGHT_REAR)

        self.joint_publisher.publish(self.joint_state)

    
    def tripod_gait(self, leg: enums.Leg):

        if self.is_moving:

            if self.trajectory_data.case_step[leg] == 0:
                self.foot_coordinate_system.leg[leg].trajectory_present_position[X] = -self.trajectory_data.amplitude_x[leg]     * cos(pi * self.trajectory_data.tick / self.trajectory_settings[enums.Gait.TRIPOD].max_ticks)
                self.foot_coordinate_system.leg[leg].trajectory_present_position[Y] = -self.trajectory_data.amplitude_y[leg]     * cos(pi * self.trajectory_data.tick / self.trajectory_settings[enums.Gait.TRIPOD].max_ticks)
                self.foot_coordinate_system.leg[leg].trajectory_present_position[Z] = abs(self.trajectory_data.amplitude_z[leg]) * sin(pi * self.trajectory_data.tick / self.trajectory_settings[enums.Gait.TRIPOD].max_ticks)


            elif self.trajectory_data.case_step[leg] == 1:
                self.foot_coordinate_system.leg[leg].trajectory_present_position[X] = self.trajectory_data.amplitude_x[leg] - 2 * self.trajectory_data.amplitude_x[leg] * self.trajectory_data.tick / self.trajectory_settings[enums.Gait.TRIPOD].max_ticks
                self.foot_coordinate_system.leg[leg].trajectory_present_position[Y] = self.trajectory_data.amplitude_y[leg] - 2 * self.trajectory_data.amplitude_y[leg] * self.trajectory_data.tick / self.trajectory_settings[enums.Gait.TRIPOD].max_ticks
                self.foot_coordinate_system.leg[leg].trajectory_present_position[Z] = 0