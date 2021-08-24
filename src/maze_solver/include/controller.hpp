#ifndef MAZE_SOLVER_CTRL_H
#define MAZE_SOLVER_CTRL_H

#include "stdio.h"
#include <ros/ros.h>

#include "adachi.hpp"
#include "logic.hpp"
#include "maze_solver.hpp"
#include "my_msg/maze.h"

using namespace std;
using namespace my_msg;

class MazeSolverCtrl {
public:
  MazeSolverCtrl(/* args */);
  ~MazeSolverCtrl();
  void init();
  void set_node_handler(const ros::NodeHandle &nh) { _nh = nh; }
  void timer_callback(const ros::TimerEvent &e);
  void set_meta_maze_data();
  void update_step_map();

  void update_sensing_result();

  bool existWall_from_premap(int x, int y, int dir);

private:
  ros::Timer timer;
  ros::NodeHandle _nh;
  ros::Publisher pub_maze_data;

  unsigned char map[MAX_MAZE_SIZE][MAX_MAZE_SIZE];
  unsigned char pre_map_data[MAX_MAZE_SIZE][MAX_MAZE_SIZE];
  unsigned int dist[MAX_MAZE_SIZE][MAX_MAZE_SIZE];
  int maze_size = 16;
  int max_step_val = 255;
  vector<point_t> goal_list;
  point_t tmp_goal_p;
  MazeSolverBaseLgc lgc;
  ego_t ego = {0};
  int global_cnt = 0;
  Adachi adachi;
  int mode = 1;
};

#endif