from pynput import keyboard
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

pressed = set()

NUMPAD_2_VK = 0x62
NUMPAD_8_VK = 0x68
NUMPAD_4_VK = 0x64
NUMPAD_6_VK = 0x66

LEFT_JOYSTICK_X = 0
LEFT_JOYSTICK_Y = 1
RIGHT_JOYSTICK_X = 3
RIGHT_JOYSTICK_Y = 4
LEFT_TRIGGER = 2
RIGHT_TRIGGER = 5
D_PAD_X = 6
D_PAD_Y = 7

LEFT_BUMPER_BUTTON = 4
RIGHT_BUMPER_BUTTON = 5
LEFT_TRIGGER_BUTTON = 6
RIGHT_TRIGGER_BUTTON = 7


class JoystickEmulator(Node):
    def __init__(self):
        super().__init__('joystick_emulator')
        self.__topic = '/joy'
        self.publisher_ = self.create_publisher(Joy, self.__topic, 10)
        self.timer = self.create_timer(0.1, self.send_message)
        self.pressed = set()

        self.get_logger().info(f'''publishing to topic {self.__topic}
Usage:

- stop program:     escape
- left joystick:    wasd
- right joystick:   ijkl
- left trigger:     q
- left bumper:      e
- right trigger:    o
- right bumper:     u
- face buttons:     1-4
- directional pad:  arrow keys
''')

    def send_message(self):
        message = Joy()

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "/dev/input/js0"

        axes = [0, 0, -1, 0, 0, -1, 0, 0]
        buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        if keyboard.KeyCode(char='w') in self.pressed:
            axes[LEFT_JOYSTICK_Y] = 1

        if keyboard.KeyCode(char='s') in self.pressed:
            axes[LEFT_JOYSTICK_Y] = -1

        if keyboard.KeyCode(char='d') in self.pressed:
            axes[LEFT_JOYSTICK_X] = 1

        if keyboard.KeyCode(char='a') in self.pressed:
            axes[LEFT_JOYSTICK_X] = -1

        if keyboard.KeyCode(char='i') in self.pressed:
            axes[RIGHT_JOYSTICK_Y] = 1

        if keyboard.KeyCode(char='k') in self.pressed:
            axes[RIGHT_JOYSTICK_Y] = -1

        if keyboard.KeyCode(char='j') in self.pressed:
            axes[RIGHT_JOYSTICK_X] = -1

        if keyboard.KeyCode(char='l') in self.pressed:
            axes[RIGHT_JOYSTICK_X] = 1

        if keyboard.KeyCode(char='q') in self.pressed:
            axes[LEFT_TRIGGER] = 1
            buttons[LEFT_TRIGGER_BUTTON] = 1

        if keyboard.KeyCode(char='o') in self.pressed:
            axes[RIGHT_TRIGGER] = 1
            buttons[RIGHT_TRIGGER_BUTTON] = 1

        if keyboard.KeyCode(char='e') in self.pressed:
            buttons[LEFT_BUMPER_BUTTON] = 1

        if keyboard.KeyCode(char='u') in self.pressed:
            buttons[RIGHT_BUMPER_BUTTON] = 1

        if keyboard.KeyCode(char='1') in self.pressed:
            buttons[0] = 1

        if keyboard.KeyCode(char='2') in self.pressed:
            buttons[1] = 1

        if keyboard.KeyCode(char='3') in self.pressed:
            buttons[2] = 1

        if keyboard.KeyCode(char='4') in self.pressed:
            buttons[3] = 1

        if keyboard.Key.down in self.pressed:
            axes[D_PAD_Y] = -1

        if keyboard.Key.up in self.pressed:
            axes[D_PAD_Y] = 1

        if keyboard.Key.left in self.pressed:
            axes[D_PAD_X] = -1

        if keyboard.Key.right in self.pressed:
            axes[D_PAD_X] = 1

        message.axes = [float(x) for x in axes]
        message.buttons = buttons

        self.publisher_.publish(message)

    def on_press(self, key):
        self.pressed.add(key)
        self.send_message()

    def on_release(self, key):
        if key == keyboard.Key.esc:
            return False
        self.pressed.remove(key)
        self.send_message()

def main(args=None):
    rclpy.init(args=args)
    joystick_emulator = JoystickEmulator()

    listener = keyboard.Listener(
        on_press=joystick_emulator.on_press,
        on_release=joystick_emulator.on_release)
    listener.start()

    try:
        rclpy.spin(joystick_emulator)
    except KeyboardInterrupt:
        pass

    joystick_emulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
