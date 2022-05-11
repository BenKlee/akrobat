import rospy
from akrobat.akrobat import Akrobat

if __name__ == '__main__':
    try:
        rospy.init_node('akrobat_main', anonymous=True)

        akrobat = Akrobat()
        akrobat.init_akrobat()

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            akrobat.run_akrobat()
            rate.sleep()
        
    except rospy.ROSInterruptException:
        pass