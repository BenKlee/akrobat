import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def publish_to_gazebo(joint_state):
    publisher = rospy.Publisher('/akrobat/joint_position_controller/command', Float64MultiArray, queue_size=1)
    publisher.publish(Float64MultiArray(data=joint_state.position))

if __name__ == '__main__':
    try:
        rospy.init_node('joint_states_to_gazebo', anonymous=True)
        rospy.Subscriber('joint_states', JointState, publish_to_gazebo)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass