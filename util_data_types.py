'''
This file contains the utiliy files required by the program. The main
classes in this file are
1. UtaisDatasetReader - Reads the obstacle data from utai dataset
2. GridGraph -  This class contains the datastructure that holds the graph 
                 data and methods to process and display graph data
'''
import os
import time

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from matplotlib.collections import LineCollection

from datastructures import *

matplotlib.rc('xtick', labelsize=25) 
matplotlib.rc('ytick', labelsize=25) 


def check_and_bound_theta(angle):
  '''
  Ensures that the theta is bounded between -pi and +pi
  '''

  if abs(angle) > 2 * np.pi:
    no_of_rotations  = angle / (2 * np.pi)
    angle = no_of_rotations - int(no_of_rotations)
    angle = angle * np.pi *2

  if abs(angle) > np.pi:
    if angle > 0:
      angle -= 2 * np.pi
    else:
      angle += 2 * np.pi
  return angle



class UtaisDatasetReader:
  '''
  Class for reading utai dataset
  '''
  def __init__(self, dataset_base_path='./data'):
    self.landmarks = self._read_landmarks(os.path.join(dataset_base_path,
                                              'ds1_Landmark_Groundtruth.dat'))

  def _read_landmarks(self, file_path):
    '''
    Reads Landmarks from landmarks file
    '''
    landmarks = {}

    with open(file_path) as landmark_file:
      landmark_string = landmark_file.read()
    landmark_datalist = landmark_string.split('\n')

    landmark_datalist = landmark_datalist[4:-1]


    for data_string in landmark_datalist:
      subject_index = int(data_string[:3])
      x_distance = float(data_string[6:16])
      y_distance = float(data_string[19:29])
      x_std_deviation = float(data_string[33:43])
      y_std_deviation = float(data_string[47:57])
      landmarks[subject_index] = { 'x_distance': x_distance,
                                   'y_distance': y_distance,
                                   'x_std_deviation': x_std_deviation,
                                   'y_std_deviation': y_std_deviation}
    return landmarks


class GridGraph:
  '''
  DataStructure for holding the graph data used in the program
  '''
  def __init__(self, step_size, x_range, y_range, obstacles_list):
    self.step_size = step_size
    self.x_range = x_range
    self.y_range = y_range
    self.obstacles_list = obstacles_list
    self.obstacle_radius = 0.3
    self.obstacle_cost = 10000
    self.fig = plt.figure()
    self.orientation_count = 0


  def intialise_display_props(self, current_position):
    '''
    Initialises initial state of the robot so that a line can be drawn from
    the present state to each subsequent state
    '''
    self.previous_x = current_position[0] 
    self.previous_y = current_position[1] 


  def get_lines_from_nodes(self, nodes_list):
    '''
    Generates a list containing the  x and y coordinates of two nodes
    This can be fed into matplotlib for plotting.
    '''

 
    lines = []
    previous_node = nodes_list[0]
    for node in nodes_list[1:]:
      lines.append([[previous_node.x, previous_node.y],
                    [node.x, node.y]])
      previous_node = node
    return np.array(lines)

  def update_obstacles_list(self, obstacle):
    self.obstacles_list.append(obstacle)

  def is_position_in_range(self, position):
    if self.x_range[0] < position[0] < self.x_range[1]:
      if self.y_range[0] < position[1] < self.y_range[1]:
        return True
    return False
 
  def is_position_colliding_with_obstacle(self, position):
    '''
     Checks if a given location is colliding with obstacles
    '''
    x1, x2 = position[0] - self.step_size/2, position[0] + self.step_size /2
    y1, y2 = position[1] - self.step_size/2, position[1] + self.step_size /2

    for pos_x, pos_y in self.obstacles_list:
      x3, x4 = pos_x - self.obstacle_radius, pos_x + self.obstacle_radius
      y3, y4 = pos_y - self.obstacle_radius, pos_y + self.obstacle_radius
      if x1 <= x3 < x2 or x1 <= x4 < x2:
        if y1 <= y3 < y2 or y1 <= y4 < y2:
          return True

      if x3 <= x1 < x4 or x3 <= x2 < x4:
        if y3 <= y1 < y4 or y3 <= y2 < y4:
          return True

    return False



  def generate_neighbour_nodes(self, current_node):
    '''
    Generates and returns all neighbor nodes to a given node
    '''
    all_neighbours_positions = [[current_node.x - self.step_size,
                                 current_node.y],
                                [current_node.x - self.step_size,
                                 current_node.y - self.step_size],
                                [current_node.x - self.step_size,
                                 current_node.y + self.step_size],
                                [current_node.x,
                                 current_node.y - self.step_size],
                                [current_node.x,
                                 current_node.y + self.step_size],
                                [current_node.x + self.step_size,
                                 current_node.y - self.step_size],
                                [current_node.x + self.step_size,
                                 current_node.y + self.step_size],
                                [current_node.x + self.step_size,
                                 current_node.y]]
    refined_positions =  filter(self.is_position_in_range, all_neighbours_positions)
    refined_positions = map(lambda x:[round(x[0], 2), round(x[1], 2)], refined_positions)
    transition_costs = []
    for pos_x, pos_y in refined_positions:
      if self.is_position_colliding_with_obstacle([pos_x, pos_y]):
        transition_costs.append(self.obstacle_cost)
      else:
        transition_costs.append(1)
    transition_costs = [round(cost, 2) for cost in transition_costs]
    graph_nodes = [node([pos_x, pos_y]) for (pos_x, pos_y) in refined_positions]
    return graph_nodes, transition_costs



  def draw_graph_with_path(self, path, title, is_show=True, is_path_drawn=True):
    '''
    Draws a grid graph and superimposes the path on top of this graph
    '''


    fig = self.fig
    x_start, x_end = self.x_range
    y_start, y_end = self.y_range
    x_start -= self.step_size
    x_end += self.step_size
    y_start -= self.step_size
    y_end += self.step_size
    ax = fig.gca()

    red_patch = mpatches.Patch(color='red', label='Goal Node')
    blue_patch = mpatches.Patch(color='blue', label='Start Node')
    green_patch = mpatches.Patch(color='green', label='Robot path')
    pink_patch = mpatches.Patch(color='burlywood', label='Robot Orientation')
    legend_patches = [red_patch, blue_patch, green_patch, pink_patch]

    plt.legend(handles=legend_patches, prop={'size': 30}, loc='upper right')


    start_node_x, start_node_y = [path[0].x, path[0].y]
    end_node_x, end_node_y = [path[-1].x, path[-1].y]

    x_remainder = round((start_node_x -  x_start) % self.step_size, 2)
    x_quotient = int((start_node_x -  x_start) / self.step_size)
    if x_remainder >= 0.5:
      x_start = start_node_x - x_quotient * self.step_size - 0.5 * self.step_size
    else:
      x_start = start_node_x - x_quotient * self.step_size + 0.5 * self.step_size


    y_remainder = round((start_node_y -  y_start) % self.step_size, 2)
    y_quotient = int((start_node_y -  y_start) / self.step_size)
    if y_remainder >= 0.5:
      y_start = start_node_y - y_quotient * self.step_size - 0.5 * self.step_size
    else:
      y_start = start_node_y - y_quotient * self.step_size + 0.5 * self.step_size

    if self.step_size == 1.0:
      ax.set_xticks(np.arange(x_start, x_end, self.step_size))
      ax.set_yticks(np.arange(y_start, y_end, self.step_size))
      plt.grid()

    ax.set_xlim([x_start, x_end])
    ax.set_ylim([y_start, y_end])
    plt.title(title, color='green', fontsize=40)
    plt.xlabel('X distance', color='blue', fontsize=40)
    plt.ylabel('Y distance', color='blue', fontsize=40)

    for x,y in self.obstacles_list:
      x1, x2 = x + self.obstacle_radius , x - self.obstacle_radius 
      y1, y2 = y + self.obstacle_radius , y - self.obstacle_radius 
      plt.fill_between([x1,x2], y1, y2, color='black')

    plt.fill_between([start_node_x - self.step_size / 2, start_node_x + self.step_size / 2],
                     start_node_y - self.step_size / 2, start_node_y + self.step_size / 2,
                     color='blue')

    plt.fill_between([end_node_x - self.step_size / 2, end_node_x + self.step_size / 2],
                     end_node_y - self.step_size / 2, end_node_y + self.step_size / 2,
                     color='red')
    #plt.text(0.5, 0.5,'Path length='+str(len(path)), fontdict={'color': 'darkred', 'size':50})


    if is_path_drawn:
      path_lines = self.get_lines_from_nodes(path)
      path_segments = LineCollection(path_lines, linewidths=(5.0),
                                   colors='green', linestyle='solid')
      ax.add_collection(path_segments)

    plt.axis('equal')
    if (is_show):
      plt.show()


  def draw_line_to_new_point(self, current_position, is_pause=False):
    '''
     For the new point given, a line is drawn from the previous point to the 
     new point
    '''
    step_size = 0.1
    current_x, current_y, current_theta = current_position
    previous_x, previous_y = self.previous_x, self.previous_y
    color = 'red'
    #plt.scatter(current_x, current_y, color)
    xvalues = np.array([previous_x, current_x])
    yvalues = np.array([previous_y, current_y])
    plt.plot(xvalues, yvalues, 'ro-', linewidth = 0.001, color='green')
    if step_size == 0.1 and self.orientation_count >=20:
      x_direct, y_direct = [0.001 * np.cos(current_theta), 0.001 * np.sin(current_theta)]
      plt.quiver(current_x, current_y, x_direct, y_direct, color='burlywood', width=0.008)
      self.orientation_count = 0

    if is_pause:
      plt.pause(0.1)
    self.previous_x, self.previous_y = current_x,  current_y
    self.orientation_count += 1

  def find_closest_node_in_the_graph(self, start_point, point):
    '''
    When the robot lands up in a state not existing in a graph,this function
    converts the given state to the state in the graph that is nearest to
    the given state
    '''
    x_pos, y_pos = [round(point[0], 2), round(point[1], 2)]
    start_x, start_y = start_point

    if x_pos > start_x:
      x_diff = x_pos - start_x
      x_offset = x_diff % self.step_size
      if x_offset > self.step_size / 2:
        x_pos += self.step_size
      x_pos -= x_offset

    if x_pos <= start_x:
      x_diff = start_x - x_pos
      x_offset = x_diff % self.step_size
      if x_offset > self.step_size / 2:
        x_pos -= self.step_size
      x_pos += x_offset


    if y_pos > start_y:
      y_diff = y_pos - start_y
      y_offset = y_diff % self.step_size
      if y_offset > self.step_size / 2:
        y_pos += self.step_size
      y_pos -= y_offset

    if y_pos <= start_y:
      y_diff = start_y - y_pos
      y_offset = y_diff % self.step_size
      if y_offset > self.step_size / 2:
        y_pos -= self.step_size
      y_pos += y_offset
    x_pos = round(x_pos, 2)
    y_pos = round(y_pos, 2)
    return x_pos, y_pos








if __name__=='__main__':
  utai_reader = UtaisDatasetReader()
  print 'finish'
