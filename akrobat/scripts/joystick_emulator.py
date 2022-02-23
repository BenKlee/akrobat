from pynput import keyboard
import rospy
from sensor_msgs.msg import Joy

pressed = set()

NUMPAD_2_VK = '0x62'
NUMPAD_8_VK = '0x68'
NUMPAD_4_VK = '0x64'
NUMPAD_6_VK = '0x66'

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




publisher = rospy.Publisher('joy', Joy)

def send_message():
    message = Joy()

    message.header.stamp = rospy.Time.now()
    message.header.frame_id = "/dev/input/js0"

    axes = [0,0,-1,0,0,-1,0,0]
    buttons = [0,0,0,0,0,0,0,0,0,0,0,0,0]

    if keyboard.KeyCode(char='w') in pressed:
        axes[LEFT_JOYSTICK_Y] = 1

    if keyboard.KeyCode(char='s') in pressed:
        axes[LEFT_JOYSTICK_Y] = -1

    if keyboard.KeyCode(char='d') in pressed:
        axes[LEFT_JOYSTICK_X] = 1

    if keyboard.KeyCode(char='a') in pressed:
        axes[LEFT_JOYSTICK_X] = -1

    if keyboard.Key.up in pressed:
        axes[RIGHT_JOYSTICK_Y] = 1
    
    if keyboard.Key.down in pressed:
        axes[RIGHT_JOYSTICK_Y] = -1
    
    if keyboard.Key.left in pressed:
        axes[RIGHT_JOYSTICK_X] = -1
    
    if keyboard.Key.right in pressed:
        axes[RIGHT_JOYSTICK_X] = 1

    if keyboard.KeyCode(char='l') in pressed:
        axes[LEFT_TRIGGER] = 1
        buttons[LEFT_TRIGGER_BUTTON] = 1
    
    if keyboard.KeyCode(char='r') in pressed:
        axes[RIGHT_TRIGGER] = 1
        buttons[RIGHT_TRIGGER_BUTTON] = 1
    
    if keyboard.KeyCode(char='q') in pressed:
        buttons[LEFT_BUMPER_BUTTON] = 1

    if keyboard.KeyCode(char='e') in pressed:
        buttons[RIGHT_BUMPER_BUTTON] = 1

    if keyboard.KeyCode(char='1') in pressed:
        buttons[0] = 1

    if keyboard.KeyCode(char='2') in pressed:
        buttons[1] = 1

    if keyboard.KeyCode(char='3') in pressed:
        buttons[2] = 1

    if keyboard.KeyCode(char='4') in pressed:
        buttons[3] = 1

    if keyboard.KeyCode(vk=int('0x62', 16)) in pressed:
        axes[D_PAD_Y] = -1

    if keyboard.KeyCode(vk=int('0x68', 16)) in pressed:
        axes[D_PAD_Y] = 1

    if keyboard.KeyCode(vk=int('0x64', 16)) in pressed:
        axes[D_PAD_X] = -1

    if keyboard.KeyCode(vk=int('0x66', 16)) in pressed:
        axes[D_PAD_X] = 1


    message.axes = axes
    message.buttons = buttons

    publisher.publish(message)

def on_press(key):
    pressed.add(key)
    send_message()

def on_release(key):
    if key == keyboard.Key.esc:
        return False
    pressed.remove(key)
    send_message()

if __name__ == '__main__':
    try:
        rospy.init_node('joystick_emulator', anonymous=True)

        print('''Usage:

- stop program:     escape
- left joystick:    wasd
- right joystick:   arrow keys
- left trigger:     l
- right trigger:    r
- left bumper:      q
- right bumper:     e
- face buttons:     1-4
- directional pad:  numpad 4,6,8,2 as left, right, up, down
''')

        listener = keyboard.Listener(
            on_press=on_press,
            on_release=on_release)
        listener.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

