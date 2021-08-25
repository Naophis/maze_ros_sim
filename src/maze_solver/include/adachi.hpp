#ifndef Adachi_H
#define Adachi_H

#include "logic.hpp"
#include "maze_solver.hpp"
#include "stdio.h"
#include <ros/ros.h>
#include <unordered_map>
#include <vector>

using namespace std;

class Adachi {
private:
  char next_direction;
  int dist_val;
  // void setNextDirection(int x, int y, int x2, int y2, int dir, int &next_dir,
  //                       int &val);
  void setNextDirection(int x2, int y2, int dir, int &next_dir, int &val);
  bool goaled = false;
  bool goal_startpos_lock = false;
  point_t next_goal_pt;
  int limit = 32;
  int limit2 = 25;

public:
  Adachi(/* args */);
  ~Adachi();
  void set_logic(MazeSolverBaseLgc *_logic);
  void set_ego(ego_t *_ego);
  int detect_next_direction();
  int get_next_motion(int next_dir);
  void get_next_pos(int next_dir);
  int exec();
  void back_home();
  ego_t *ego;
  vector<point_t> pt_list;
  vector<point_t> start_pt_list;
  bool is_go_home();
  void deadEnd();
  bool is_goal(int x, int y);
  MazeSolverBaseLgc *lgc;
  int cost_mode = 0;
  unordered_map<unsigned int, unsigned char> subgoal_list;
};

#endif