from akrobat.joint import Joint
from numpy import array

class Leg():

  def __init__(self) -> None:
    self.foot_present_position = array([0,0,0])
    self.foot_initial_position = array([0,0,0])
    self.foot_global_position  = array([0,0,0])

    self.trajectory_present_position = array([0,0,0])

    self.joint_angles = Joint(0,0,0)