import rospy

from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Int16MultiArray


def create_callback(i: int):
    def callback(contacts: ContactsState):
        if len(contacts.states) == 0:
            legs_in_contact[i] = False
            return

        legs_in_contact[i] = True
        
    return callback


LEGS = 6
legs_in_contact = [False] * LEGS


if __name__ == '__main__':
    try:
        rospy.init_node('foot_contact', anonymous=True)
        publisher = rospy.Publisher('foot_contact', Int16MultiArray)


        for i in range(LEGS):
            callback = create_callback(i)
            rospy.Subscriber(f'leg_{i+1}_tibia_link_contact_sensor_state', ContactsState, callback)

        rate = rospy.Rate(60)

        while not rospy.is_shutdown():
            publisher.publish(Int16MultiArray(data=legs_in_contact))
            rate.sleep()

        
    except rospy.ROSInterruptException:
        pass