'''
This file contains the main logic files for the assignment. The important
classes in this file are as follows
1. AStarAlgorithm - Class implementing Astar algorithm
2. AStarAlgorithmOnline - Class implementing online version of AStar
3. pid_controller - class implementing the pid controller for controlling
                    robot velocities
'''
import heapq

import math
import numpy as np

from util_data_types import *
from datastructures import *


class AStarAlgorithm:
  '''
  Implementation of AStar algorithm
  '''
  def __init__(self, graph):
    self.graph = graph

  def hueristic(self, current_node, goal):
    x_diff = abs(current_node.x - goal.x)
    y_diff = abs(current_node.y - goal.y)

    return np.sqrt(np.linalg.norm([current_node.x - goal.x, current_node.y - goal.y]) * (min(x_diff, y_diff) + abs(x_diff - y_diff)))

  def generate_path(self, start_node, closed_set):
    '''
    Generates a list containing all nodes in the path.
    '''
    path = []
    goal = closed_set[-1]
    current_node = goal
    while current_node is not start_node:
      path.append(current_node)
      current_node = current_node.parent
    path.append(start_node)
    return path

  def find_a_star_path(self, start_pos, goal_pos, is_offline):
    '''
    Finds the Astar path
    '''
    open_set = []
    closed_set = []
    start_node = node(start_pos, cost=0)
    goal_node = node(goal_pos, cost=0)
    open_set.append(start_node)


    while goal_node not in closed_set:
      heapq.heapify(open_set)
      chosen_node = heapq.heappop(open_set)
      closed_set.append(chosen_node)
      neighbour_nodes, transition_costs = self.graph.generate_neighbour_nodes(chosen_node)

      if(not is_offline):
        open_set = []
      for neigh_node, trans_cost in zip(neighbour_nodes, transition_costs):
        if neigh_node not in closed_set and neigh_node not in open_set:
          neigh_node.cost = chosen_node.cost + trans_cost + self.hueristic(neigh_node, goal_node)
          open_set.append(neigh_node)
          neigh_node.parent = chosen_node

     # If there is no path to the goal
      if not open_set:
        break
    path = self.generate_path(start_node, closed_set)
    return list(reversed(path))

def estimate_distance_and_angle(robot_state, target_position):
  '''
   Gives the distance and orientation of a point for the given robot state
   It is the same as measurement model
  '''
  x, y, theta = robot_state
  rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                              [np.sin(theta), np.cos(theta)]], dtype=np.float64)
  inv_rotation = rotation_matrix.T
  translation = -inv_rotation.dot([x, y])
  vector_in_robot_frame  = rotation_matrix.T.dot(target_position) - rotation_matrix.T.dot([x, y]) 
  distance = np.linalg.norm(vector_in_robot_frame)
  angle = np.arctan2(vector_in_robot_frame[1], vector_in_robot_frame[0])
  angle = check_and_bound_theta(angle)
  return [distance, angle]


def calculate_next_state(current_state, velocities, time_step=0.1):
  '''
  Motion model. Here a simple unicycle model is used.
  '''
  linear_velocity, angular_velocity = velocities
  position_x, position_y, angle = current_state
  new_position_x = position_x + np.cos(angle) * linear_velocity * time_step
  new_position_y = position_y + np.sin(angle) * linear_velocity * time_step
  new_angle = angle + angular_velocity * time_step
  new_angle = check_and_bound_theta(new_angle) 
  state = np.array([new_position_x, new_position_y, new_angle])
  #position_error = 0
  position_error = 0.001
  true_x, true_y = np.random.normal(state[:2], position_error)
  #angle_error = 0
  angle_error = 0.002
  true_theta = np.random.normal(state[2], angle_error)
  return [true_x, true_y, true_theta]



class AStarAlgorithmOnline:
  '''
   Online version of the AStar algorithm
  '''
  def __init__(self, graph, start_point, goal_point):
    self.graph = graph
    self.start_point = start_point
    self.open_set = [node(start_point)]
    self.goal_node = node(goal_point)
    self.closed_set = []

  def hueristic(self, current_node, goal):
    '''
    Hueristic function used for evaluating nodes
    '''

    x_diff = abs(current_node.x - goal.x)
    y_diff = abs(current_node.y - goal.y)
    return np.sqrt(np.linalg.norm([current_node.x - goal.x, current_node.y - goal.y]) * (min(x_diff, y_diff) + abs(x_diff - y_diff)))

  def generate_path(self, start_node, closed_set):
    '''
    Generates a list containing a sequence of nodes that lie in the path
    that was found
    '''
    path = []
    goal = closed_set[-1]
    current_node = goal
    while current_node is not start_node:
      path.append(current_node)
      current_node = current_node.parent
    path.append(start_node)
    return path

  def find_the_next_node(self, start_pos):
    '''
    Returns the next node to visit. Evaluates all neighbors of the
    current state and returns its cheapest neighbor
    '''
    start_pos = self.graph.find_closest_node_in_the_graph(self.start_point,
                                                         start_pos)
    start_node = node(start_pos, cost=0)
    chosen_node = start_node
    self.closed_set.append(node(start_pos))
    neighbour_nodes, transition_costs = self.graph.generate_neighbour_nodes(chosen_node)

    self.open_set = []
    for neigh_node, trans_cost in zip(neighbour_nodes, transition_costs):
      if neigh_node not in self.closed_set and neigh_node not in self.open_set:
        neigh_node.cost = chosen_node.cost + trans_cost + self.hueristic(neigh_node,
                                                                         self.goal_node)
        self.open_set.append(neigh_node)
        neigh_node.parent = chosen_node

    heapq.heapify(self.open_set)
    chosen_node = heapq.heappop(self.open_set)

    if chosen_node == self.goal_node:
      return True, chosen_node
    else:
      return False, chosen_node



class pid_controller:
  '''
  PID controller for controlling robot velocities
  '''
  def __init__(self, kparameters, accelerations, time_step):
    self.kdw, self.kv, self.kpw, self.kiw = kparameters
    self.max_av, self.max_wv = accelerations
    self.time_step = time_step
    self.previous_error = 0
    self.total_error = 0

  def execute_one_robot_command(self, goal_position, current_state):
      '''
      Executes one control sequence
      '''
      x_goal, y_goal = goal_position
      distance, angle =  estimate_distance_and_angle([current_state.robot_pose_x,
                                                      current_state.robot_pose_y,
                                                      current_state.robot_pose_angle],
                                                      [x_goal, y_goal])
      required_v = distance / self.time_step
      current_error = angle
      derror = (current_error - self.previous_error) / self.time_step
      self.total_error += current_error
      self.previous_error = current_error
      ierror = self.total_error * self.time_step
      required_w = self.kpw * current_error / self.time_step + self.kiw * ierror +\
                   self.kdw * derror

      if angle > current_state.robot_pose_angle:
        next_w = min(current_state.angular_velocity + self.max_wv * self.time_step,
                       required_w)
      else:
        next_w = max(current_state.angular_velocity - self.max_wv * self.time_step,
                       required_w)
 


      if required_v > current_state.linear_velocity:
        next_v = min(current_state.linear_velocity + self.max_av * self.time_step,
                     required_v)
      else:
        next_v = max(current_state.linear_velocity - self.max_av * self.time_step,
                     required_v)
      next_v = self.kv * next_v
      next_position = calculate_next_state([current_state.robot_pose_x,
                                            current_state.robot_pose_y,
                                            current_state.robot_pose_angle],
                                            [next_v, next_w])
      current_state.angular_velocity = next_w
      current_state.linear_velocity = next_v
      current_state.robot_pose_angle = next_position[2]
      current_state.robot_pose_x = next_position[0]
      current_state.robot_pose_y = next_position[1]

      return current_state


def run_a_star_algo(start_pos, goal_pos, step_size, obstacles, is_offline=True):
  '''
  Function to run Plain AStar algorithm
  '''

  x_range = [-5, 5]
  y_range = [-6, 6]
  obstacles_list = []
  env_graph = GridGraph(step_size, x_range, y_range, obstacles)
  a_star_algo = AStarAlgorithm(env_graph)
  path = a_star_algo.find_a_star_path(start_pos, goal_pos, is_offline)
  return path, env_graph


def run_robot_wth_controller_preplanned_path(kparameters, start_pos,
                                             goal_pos, step_size):
  '''
   Runs PID controller to follow the path generated by AStar
  '''

  kdw, kv, kpw, kiw = kparameters
  max_av = 0.288
  max_wv = 5.579
  time_step = 0.1
  robot_controller = pid_controller(kparameters, [max_av, max_wv], time_step)


  utai_reader = UtaisDatasetReader()
  landmarks= utai_reader.landmarks
  obstacles = [[landmark['x_distance'], landmark['y_distance']]
                for key, landmark in landmarks.items() if landmark]
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles,
                                    is_offline=False)
  env_graph.draw_graph_with_path(path, 'Part-B.9 Running robot controller with online paths',
                                 is_show=False, is_path_drawn=False)

  current_node = path[0]
  current_state = robot_state([current_node.x, current_node.y, -np.pi/2],
                            [0, 0])
  env_graph.intialise_display_props([current_node.x, current_node.y])
  distance_threshold = 0.1
  is_app_running = True



  for target_node in path[1:]:
    x_goal, y_goal = target_node.x, target_node.y
    distance = 1
    while distance > distance_threshold and is_app_running:
      next_state = robot_controller.execute_one_robot_command([x_goal, y_goal],
                                                               current_state)
      distance = np.linalg.norm([x_goal - current_state.robot_pose_x,
                                 y_goal - current_state.robot_pose_y])
      try:
        env_graph.draw_line_to_new_point([next_state.robot_pose_x,
                                          next_state.robot_pose_y,
                                          next_state.robot_pose_angle])
      except:
        is_app_running = False
        break

  try:
    plt.show()
  except:
    pass



def run_robot_wth_controller_online_path(kparameters, start_pos,
                                             goal_pos, step_size, title):
  '''
  Runs the controller while generating paths on the fly
  '''

  kdw, kv, kpw, kiw = kparameters
  max_av = 0.288
  max_wv = 5.579
  time_step = 0.1
  robot_controller = pid_controller(kparameters, [max_av, max_wv], time_step)
  utai_reader = UtaisDatasetReader()
  landmarks= utai_reader.landmarks
  obstacles = [[landmark['x_distance'], landmark['y_distance']]
                for key, landmark in landmarks.items() if landmark]

  x_range = [-5, 5]
  y_range = [-6, 6]
  env_graph = GridGraph(step_size, x_range, y_range, obstacles)
  path = [node([start_pos[0], start_pos[1]])]
  end_node = node([goal_pos[0], goal_pos[1]])
  path.append(end_node)
  env_graph.draw_graph_with_path(path, title = title,
                                 is_show=False, is_path_drawn=False)
  current_node = node([start_pos[0], start_pos[1]])
  current_state = robot_state([current_node.x, current_node.y, -np.pi/2],
                            [0, 0])
  env_graph.intialise_display_props([current_node.x, current_node.y])
  distance_threshold = 0.08

  is_algo_ended = False
  is_app_running = True

  a_star_path = AStarAlgorithmOnline(env_graph, start_pos, goal_pos)

  while not is_algo_ended:
    is_algo_ended, next_node = a_star_path.find_the_next_node([current_state.robot_pose_x,
                                                        current_state.robot_pose_y])
    x_goal, y_goal = [next_node.x, next_node.y]
    distance = 1
    while distance > distance_threshold and is_app_running:
      next_state = robot_controller.execute_one_robot_command([x_goal, y_goal],
                                                               current_state)
      if next_state.robot_pose_x < 2.5:
        pass
      distance = np.linalg.norm([x_goal - current_state.robot_pose_x,
                                 y_goal - current_state.robot_pose_y])
      try:
        env_graph.draw_line_to_new_point([next_state.robot_pose_x,
                                          next_state.robot_pose_y,
                                          next_state.robot_pose_angle], is_pause=False)
      except:
        is_app_running = False
        break

  try:
    plt.show()
  except:
    pass

