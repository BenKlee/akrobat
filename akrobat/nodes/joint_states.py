from cmath import pi
from typing import Callable
import rospy
from math import sin, cos

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Int16MultiArray

mX1_limit = 0.548
mX2_limit = 1.57
mX3_limit = 1.57


joints = [
    'm11',
    'm12',
    'm13',
    'm21',
    'm22',
    'm23',
    'm31',
    'm32',
    'm33',
    'm41',
    'm42',
    'm43',
    'm51',
    'm52',
    'm53',
    'm61',
    'm62',
    'm63',
]

position = [-1] * len(joints)
velocity = [0] * len(joints)
effort   = [0] * len(joints)

class Motor:

  def __init__(self, index: int, movement_function: Callable) -> None:
    self.index = index
    self.movement_function = movement_function
    self.movement_offset = 0

class Leg:

  def __init__(self, alpha: Motor, beta: Motor, gamma: Motor) -> None:
    self.alpha = alpha
    self.beta = beta
    self.gamma = gamma
    self.grounded = True


def set_ground_contact(contacts: Int16MultiArray):
  for i in range(len(contacts.data)):
    legs[i].grounded = bool(contacts.data[i])

legs = []

def even(int: int):
  return int%2==0

for leg_index in range(6):
  if even(leg_index):
    legs.append(Leg(Motor(leg_index*3+0, lambda x: 0.548*sin(x)), Motor(leg_index*3+1, lambda x: 1.57/2*cos(x)), Motor(leg_index*3+2, lambda: '')))
  else:
    legs.append(Leg(Motor(leg_index*3+0, lambda x: -0.548*sin(x)), Motor(leg_index*3+1, lambda x: 1.57/2*cos(x)), Motor(leg_index*3+2, lambda: '')))

  if (leg_index+1) in (2,3,6):
    legs[leg_index].alpha.movement_offset = pi
    legs[leg_index].beta.movement_offset = pi


def set_mX1_motors(value, offset = True):
  for i in range(len(joints)):
    if i%3 == 0:
      position[i] = sin(value) if not (offset and i in (6,9)) else sin(value + pi)

def set_mX2_motors(value, offset = True):
  for i in range(len(joints)):
    if i%3 == 1:
      position[i] = -cos(value) if not (offset and i in (4,7,16)) else -cos(value + pi)


def publish():
  publisher = rospy.Publisher('/joint_states', JointState, queue_size=1)

  radian = 0
  rate = rospy.Rate(60)

  rospy.Subscriber('foot_contact', Int16MultiArray, set_ground_contact)


  while not rospy.is_shutdown():
    time = rospy.Time.now()

    for i in range(len(legs)):
      leg = legs[i]
      position_alpha = leg.alpha.movement_function(radian + leg.alpha.movement_offset)
      # print(f'leg {i}-alpha: {position_alpha}')
      position[leg.alpha.index] = position_alpha

      # if not leg.grounded or legs[(i+1)%6].grounded:
      # position[leg.beta.index] = leg.beta.movement_function(radian + leg.beta.movement_offset)
      position[leg.beta.index] = -1.57/2

    radian += pi/60
    radian %= 2*pi

    publisher.publish(JointState(
            header=Header(stamp=time),
            name=joints,
            position=position,
            velocity=velocity,
            effort=effort
    ))
    rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('joint_states', anonymous=True)
        publish()
        
    except rospy.ROSInterruptException:
        pass