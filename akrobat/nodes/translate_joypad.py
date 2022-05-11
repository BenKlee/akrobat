from enum import Enum
from akrobat.enums import Gait
import rospy
from sensor_msgs.msg import Joy
from akrobat.msg import movement

MAX_INT = 32767
JOYSTICK_DEADZONE = .2

class ModeRightJoystick(Enum):
    default     = 0
    shift       = 1
    roll_pitch  = 2
    yaw         = 3
    level       = 4


class Mode(Enum):
    navigate    = 0
    work        = 1


class Macro(Enum):
    default                 = 0
    feet                    = 1
    orientate_shift         = 2
    orientate_yaw           = 3
    orientate_level         = 4
    orientate_roll_pitch    = 5
    start                   = 6
    shutdown                = 7


class Controller(Enum):
    xbox_360_wired  = 'Microsoft Xbox 360 Wired Controller'
    f710_gamepad    = 'Logitech Gamepad F710'
    dualshock_4     = 'Dualshock 4'


class Listener():

    def __init__(self) -> None:
        self.axes = None
        self.buttons = None
        self.walking_mode = ''
        self.controller_type = 'DEFAULT'

    def joypad_callback(self, msg: Joy):
        amount_axes = len(msg.axes)
        amount_buttons = len(msg.buttons)

        if amount_buttons == 11 and amount_axes == 8:
            self.controller_type = Controller.xbox_360_wired
        elif amount_buttons == 12 and amount_axes == 6:
            self.controller_type = Controller.f710_gamepad
        elif amount_buttons == 13 and amount_axes == 8:
            self.controller_type = Controller.dualshock_4

        self.axes = msg.axes
        self.buttons = msg.buttons

    def movement_callback(self, msg: movement):
        self.walking_mode = msg.walking_mode


if __name__ == '__main__':
    try:
        rospy.init_node('translate_joypad', anonymous=True)

        listener = Listener()
        
        rospy.Subscriber('joy', Joy, listener.joypad_callback, queue_size=1000)
        rospy.Subscriber('movements', movement, listener.movement_callback, queue_size=1000)

        movement_publisher = rospy.Publisher('movements', movement, queue_size=1000)

        rate = rospy.Rate(20)

        mode = Mode.navigate
        gait = Gait.TRIPOD
        joystick_mode = ModeRightJoystick.default
        macro = Macro.start
        buttons_pressed = [False] * 11


        while not rospy.is_shutdown():

            if listener.walking_mode is Gait.RESET:
                gait = Gait.RESET

            axes = listener.axes
            buttons = listener.buttons
            controller_type = listener.controller_type
            msg = movement()

            controller_names = []

            for controller in Controller:
                controller_names.append(controller)

            if controller_type not in controller_names:
                print('Warte auf passenden Controller')
                rate.sleep()
                continue

            commands = [0] * 9

            if mode is Mode.navigate:

                # LEFT JOYSTICK
                if abs(axes[1]) > JOYSTICK_DEADZONE:
                    commands[0] = axes[1] * MAX_INT
                
                if abs(axes[0]) > JOYSTICK_DEADZONE:
                    commands[1] = axes[0] * MAX_INT

                # RIGHT JOYSTICK
                if joystick_mode is not ModeRightJoystick.default:

                    if joystick_mode is ModeRightJoystick.shift:

                        if abs(axes[4]) > JOYSTICK_DEADZONE:
                            commands[6] = axes[4] * MAX_INT

                        if abs(axes[3]) > JOYSTICK_DEADZONE:
                            commands[7] = axes[3] * MAX_INT


                    if joystick_mode is ModeRightJoystick.roll_pitch:

                        if abs(axes[4]) > JOYSTICK_DEADZONE:
                            commands[4] = axes[4] * MAX_INT

                        if abs(axes[3]) > JOYSTICK_DEADZONE:
                            commands[5] = axes[3] * MAX_INT

                    
                    if joystick_mode is ModeRightJoystick.yaw:

                        if abs(axes[3]) > JOYSTICK_DEADZONE:
                            commands[3] = axes[3] * MAX_INT

                        if abs(axes[4]) > JOYSTICK_DEADZONE:
                            commands[8] = axes[4] * MAX_INT
                

                    if joystick_mode is ModeRightJoystick.level:

                        if abs(axes[4]) > JOYSTICK_DEADZONE:
                            commands[8] = axes[4] * MAX_INT


                # right and left trigger
                if abs(axes[2] - axes[5]) > JOYSTICK_DEADZONE:
                    commands[2] = ((axes[2] - axes[5]) * MAX_INT) / 2

                # digital joystick
                if axes[7] > 0:
                    macro = Macro.feet
                else:
                    macro = Macro.default

                # A button
                if buttons[0] == 1:
                    joystick_mode = ModeRightJoystick.shift

                # B button
                if buttons[1] == 1:
                    joystick_mode = ModeRightJoystick.roll_pitch

                # X button
                if buttons[2] == 1:
                    joystick_mode = ModeRightJoystick.level

                # Y button
                if buttons[3] == 1:
                    joystick_mode = ModeRightJoystick.yaw

                # left bumper
                if buttons[4] == 1:
                    gait_names = Gait._member_names_
                    gait_index = gait_names.index(gait.name)
                    new_gait_name = gait_names[(gait_index+1) % len(Gait)]
                    gait = Gait[new_gait_name]

                # start button
                if buttons[7] == 1:
                    macro.start

                # back button
                if buttons[6] == 1:
                    mode = Mode((mode+1)%len(Mode))
                
                # L3 and R3 button (press joysticks)
                if buttons[9] == 1 and buttons[10] == 1:
                    macro = Macro.shutdown
                

            elif mode is Mode.work:

                # LEFT JOYSTICK
                if abs(axes[1]) > JOYSTICK_DEADZONE:
                    commands[6] = axes[1] * MAX_INT
                
                if abs(axes[0]) > JOYSTICK_DEADZONE:
                    commands[7] = axes[0] * MAX_INT


                # RIGHT JOYSTICK
                if abs(axes[4]) > JOYSTICK_DEADZONE:
                    commands[4] = axes[4] * MAX_INT
                
                if abs(axes[3]) > JOYSTICK_DEADZONE:
                    commands[5] = axes[3] * MAX_INT


                # left and right trigger
                if abs(axes[2] - axes[5]) > JOYSTICK_DEADZONE:
                    commands[3] = ((axes[2] - axes[5]) * MAX_INT) / 2


                # left and right bumper
                commands[8] = (buttons[5] - buttons[4]) * MAX_INT

                # start button
                if buttons[7] == 1:
                    macro.start

                # back button
                if buttons[6] == 1:
                    mode = Mode((mode+1)%len(Mode))
                
                # L3 and R3 button (press joysticks)
                if buttons[9] == 1 and buttons[10] == 1:
                    macro = Macro.shutdown
                
                
            for i in range(11):
                if buttons[i] == 1:
                    buttons_pressed[i] += 1
                else:
                    buttons_pressed[i] = 0


            # orientate macros
            if buttons_pressed[0] >= 15:

                macro = Macro.orientate_shift
                
                if buttons_pressed[0] >= 25:
                    buttons_pressed[0] = 0
                    macro = Macro.default

            elif buttons_pressed[1] >= 15:

                macro = Macro.orientate_roll_pitch
                
                if buttons_pressed[1] >= 25:
                    buttons_pressed[1] = 0
                    macro = Macro.default

            elif buttons_pressed[2] >= 15:

                macro = Macro.orientate_level
                
                if buttons_pressed[2] >= 25:
                    buttons_pressed[2] = 0
                    macro = Macro.default

            elif buttons_pressed[3] >= 15:

                macro = Macro.orientate_yaw
                
                if buttons_pressed[3] >= 25:
                    buttons_pressed[3] = 0
                    macro = Macro.default

            msg.commands = list(map(int, commands))

            msg.walking_mode = gait.name.lower()

            msg.macro = macro.name


            movement_publisher.publish(msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass