'''
This file contains the datastructures used in the program
'''
class robot_state:
  '''
  Data Structure for holding robot state
  '''
  def __init__(self, robot_pose, velocities):
    self.robot_pose_x = robot_pose[0]
    self.robot_pose_y = robot_pose[1]
    self.robot_pose_angle = robot_pose[2]
    self.linear_velocity = velocities[0]
    self.angular_velocity = velocities[1]


class node:
  '''
  DataStructure for a graph node
  '''
  def __init__(self, position, cost=0):
    self.x, self.y = position
    self.parent = None
    self.cost = cost
  def __eq__(self, other):
    if self.x == other.x and self.y == other.y:
      return True
    return False

  def __lt__(self, other):
    return self.cost < other.cost
