import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64



def publish_to_gazebo(joint_state):
    for name, value in zip(joint_state.name, joint_state.position):
        publisher = rospy.Publisher('/akrobat/' + name + '_position_controller/command', Float64, queue_size=1)
        publisher.publish(value)

if __name__ == '__main__':
    try:
        rospy.init_node('joint_states_to_gazebo', anonymous=True)
        rospy.Subscriber('joint_states', JointState, publish_to_gazebo)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass