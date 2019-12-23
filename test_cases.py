'''
This file just contains the test cases to be run to produce the outputs
that have been stated in the assignment
Function  names are self-explanatory for the test cases they run
'''

from a_star_algorithm import *
import time

def run_offline_planning_test__with_step_size_1(obstacles):

  start_pos = [0.5, -1.5]
  goal_pos = [0.5, 1.5]
  step_size = 1.0
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles)
  env_graph.draw_graph_with_path(path, 'Part-A.3 Offline Paths with step size 1')

  start_pos = [4.5, 3.5]
  goal_pos = [4.5, -1.5]
  step_size = 1.0
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles)
  env_graph.draw_graph_with_path(path, 'Part-A.3 Offline Paths with step size 1')

  start_pos = [-0.5, 5.5]
  goal_pos = [1.5, -3.5]
  step_size = 1.0
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles)
  env_graph.draw_graph_with_path(path, 'Part-A.3 Offline Paths with step size 1')


def run_online_planning_test__with_step_size_1(obstacles):

  start_pos = [0.5, -1.5]
  goal_pos = [0.5, 1.5]
  step_size = 1.0
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles, is_offline=False)
  env_graph.draw_graph_with_path(path, 'Part-A.5 Online Paths with step size 1')

  start_pos = [4.5, 3.5]
  goal_pos = [4.5, -1.5]
  step_size = 1.0
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles, is_offline=False)
  env_graph.draw_graph_with_path(path, 'Part-A.5 Online Paths with step size 1')

  start_pos = [-0.5, 5.5]
  goal_pos = [1.5, -3.5]
  step_size = 1.0
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles,is_offline=False)
  env_graph.draw_graph_with_path(path, 'Part-A.5 Online Paths with step size 1')

def run_online_planning_test(obstacles):

  start_pos = [2.45, -3.55]
  goal_pos = [0.95, -1.55]
  step_size = 0.1

  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles,
                                    is_offline=False)
  env_graph.draw_graph_with_path(path, 'Part-A.7 Online Paths with step size 0.1')


  start_pos = [4.95, -0.05]
  goal_pos = [2.45, 0.25]
  step_size = 0.1
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles,
                                    is_offline=False)
  env_graph.draw_graph_with_path(path, 'Part-A.7 Online Paths with step size 0.1')

  start_pos = [-0.55, 1.45]
  goal_pos = [1.95, 3.95]
  step_size = 0.1
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles,
                                    is_offline=False)
  env_graph.draw_graph_with_path(path, 'Part-A.7 Online Paths with step size 0.1')



def run_robot_controller_with_preplanned_online_path(obstacles):

  kparameters = [0.75, 0.75, 0.3, 0.7]
  step_size = 0.1
  start_pos = [2.45, -3.55]
  goal_pos = [0.95, -1.55]
  run_robot_wth_controller_preplanned_path(kparameters, start_pos, goal_pos, step_size)

  start_pos = [4.95, -0.05]
  goal_pos = [2.45, 0.25]
  run_robot_wth_controller_preplanned_path(kparameters, start_pos, goal_pos, step_size)

  start_pos = [-0.55, 1.45]
  goal_pos = [1.95, 3.95]
  run_robot_wth_controller_preplanned_path(kparameters, start_pos, goal_pos, step_size)



def run_robot_wth_controller_with_on_the_fly_paths(obstacles):

  kparameters = [0.3, 0.75, 0.75, 0.7]
  title = 'Part-B.10 Running robot controller paths generated on the fly'
  start_pos = [2.45, -3.55]
  goal_pos = [0.95, -1.55]
  step_size = 0.1

  run_robot_wth_controller_online_path(kparameters, start_pos, goal_pos, step_size,
                                       title)

  start_pos = [4.95, -0.05]
  goal_pos = [2.45, 0.25]
  run_robot_wth_controller_online_path(kparameters, start_pos, goal_pos, step_size,
                                       title)


  start_pos = [-0.55, 1.45]
  goal_pos = [1.95, 3.95]
  run_robot_wth_controller_online_path(kparameters, start_pos, goal_pos, step_size,
                                       title)

def coarse_grid_fine_grid_comparison(obstacles):



  start_pos = [0.5, -1.5]
  goal_pos = [0.5, 1.5]
  step_size = 1.0

  title = 'Part-B.11 Coarse-Grid vs Fine Grid comparison \n Step size 1'

  kparameters = [0.3, 0.75, 0.75, 0.7]
  run_robot_wth_controller_online_path(kparameters, start_pos, goal_pos, step_size,
                                       title)
  step_size = 0.1
  title = 'Part-B.11 Coarse-Grid vs Fine Grid comparison\n Step size 0.1'
  run_robot_wth_controller_online_path(kparameters, start_pos, goal_pos, step_size,
                                       title)


  start_pos = [4.5, 3.5]
  goal_pos = [4.5, -1.5]
  step_size = 1.0

  title = 'Part-B.11 Coarse-Grid vs Fine Grid comparison\n Step size 1'
  run_robot_wth_controller_online_path(kparameters, start_pos, goal_pos, step_size,
                                       title)
  step_size = 0.1
  title = 'Part-B.11 Coarse-Grid vs Fine Grid comparison\n Step size 0.1'
  run_robot_wth_controller_online_path(kparameters, start_pos, goal_pos, step_size,
                                       title)

  start_pos = [-0.5, 5.5]
  goal_pos = [1.5, -3.5]
  step_size = 1.0

  title = 'Part-B.11 Coarse-Grid vs Fine Grid comparison Step size 1'

  run_robot_wth_controller_online_path(kparameters, start_pos, goal_pos, step_size,
                                       title)
  step_size = 0.1

  title = 'Part-B.11 Coarse-Grid vs Fine Grid comparison\n Step size 0.1'
  run_robot_wth_controller_online_path(kparameters, start_pos, goal_pos, step_size,
                                       title)

def time_test(obstacles): 


  start_pos = [-0.5, 5.5]
  goal_pos = [1.5, -3.5]
  step_size = 1.0
  start = time.time()
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles)
  print('step size: 1:' + 'total_time =', time.time() - start)


  start_pos = [-0.5, 5.5]
  goal_pos = [1.5, -3.5]
  step_size = 1.0
  start = time.time()
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles)
  print('step size: 1:' + 'total_time =', time.time() - start)


  step_size = 0.75
  start = time.time()
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles)
  print('step size: 0.75:' + 'total_time =', time.time() - start)


  step_size = 0.5
  start = time.time()
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles)
  print('step size: 0.5:' + 'total_time =', time.time() - start)

  step_size = 0.25
  start = time.time()
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles)
  print('step size: 0.25:' + 'total_time =', time.time() - start)

  step_size = 0.1
  start = time.time()
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles)
  print('step size 0.1' +' total_time =', time.time() - start)

  # step_size = 0.01
  # start = time.time()
  # path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles)
  # print('total_time =', time.time() - start)


  # step_size = 0.001
  # start = time.time()
  # path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles)
  # print('total_time =', time.time() - start)
  # #env_graph.draw_graph_with_path(path, 'Part-A.3 Offline Paths with step size 1')


def completeness_test(obstacles):

  goal_pos = [0.5, -1.5]
  start_pos =  [0.5, 1.5]
  step_size = 1.0
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles)
  env_graph.draw_graph_with_path(path, 'Part-A.3 Offline Paths with step size 1')

  goal_pos = [4.5, 3.5]
  start_pos = [4.5, -1.5]
  step_size = 1.0
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles)
  env_graph.draw_graph_with_path(path, 'Part-A.3 Offline Paths with step size 1')

  goal_pos = [-0.5, 5.5]
  start_pos = [1.5, -3.5]
  step_size = 1.0
  path, env_graph = run_a_star_algo(start_pos, goal_pos, step_size, obstacles)
  env_graph.draw_graph_with_path(path, 'Part-A.3 Offline Paths with step size 1')

def k_value_experimentation_for_pid(obstacles):


  start_pos = [4.95, -0.05]
  goal_pos = [2.45, 0.25]
  step_size = 0.1

  title = 'Kvalue experimentation - kwp=0.85, kwi=0.75, kwd=0.3 kvp=0.7'

  kparameters = [0.85, 0.75, 0.3, 0.7]
  run_robot_wth_controller_online_path(kparameters, start_pos, goal_pos, step_size,
                                       title)

