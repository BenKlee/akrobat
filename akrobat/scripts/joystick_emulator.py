import rospy

from sensor_msgs.msg import Joy

amountButtons = 13
amountAxis = 8

def emulate():
    rospy.init_node('joystick_emulator', anonymous=True)
    publisher = rospy.Publisher('/joy', Joy)

    axes =[]
    for i in range(amountAxis):
        axes.append(0)

    axes[1] = 1

    buttons =[] 
    for i in range(amountButtons):
        buttons.append(0)

    msg = Joy()

    msg.axes = axes
    msg.buttons = buttons

    msg.header.frame_id = '/dev/input/js0'


    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        publisher.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        emulate()
    except rospy.ROSInterruptException:
        pass