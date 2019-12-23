'''
Master Function for running all test cases
'''
from a_star_algorithm import *
from test_cases import *



if __name__=='__main__':

  utai_reader = UtaisDatasetReader()
  landmarks= utai_reader.landmarks
  obstacles = [[landmark['x_distance'], landmark['y_distance']]
                for key, landmark in landmarks.items() if landmark]


  run_offline_planning_test__with_step_size_1(obstacles)
  run_online_planning_test__with_step_size_1(obstacles)
  run_offline_planning_test__with_step_size_1(obstacles)
  run_online_planning_test__with_step_size_1(obstacles)
  run_online_planning_test(obstacles)
  run_robot_controller_with_preplanned_online_path(obstacles)
  run_robot_wth_controller_with_on_the_fly_paths(obstacles)
  coarse_grid_fine_grid_comparison(obstacles)

