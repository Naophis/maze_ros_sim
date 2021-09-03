#ifndef TRAJECTORY_CREATOR_H
#define TRAJECTORY_CREATOR_H

#include "maze_solver.hpp"
#include "stdio.h"
#include <cmath>
#include <ros/ros.h>
#include <vector>

using namespace std;

constexpr double ROOT2 = sqrt(2);
class TrajectoryCreator {
private:
  TurnType get_turn_type(int turn_num);
  TurnDirection get_turn_dir(int turn_dir);
  float get_run_dir(Direction dir, float ang);
  float get_turn_tgt_ang(TurnType type);
  float get_turn_rad(TurnType type);
  Direction get_next_dir(Direction dir, TurnType type, TurnDirection turn_dir);
  void fix_pos(ego_odom_t &ego, TurnType type, TurnDirection turn_dir);

public:
  TrajectoryCreator(/* args */);
  ~TrajectoryCreator();
  void exec(path_struct &base_path, vector<trajectory_point_t> &trajectory);
  void exec2(path_struct &base_path, vector<trajectory_point_t> &trajectory);
};

#endif