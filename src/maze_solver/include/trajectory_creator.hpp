#ifndef TRAJECTORY_CREATOR_H
#define TRAJECTORY_CREATOR_H

#include "maze_solver.hpp"
#include "stdio.h"
#include <cmath>
#include <ros/ros.h>
#include <vector>

using namespace std;

constexpr double ROOT2 = sqrt(2);
constexpr double dt = 0.001;
class TrajectoryCreator {
private:
  TurnType get_turn_type(int turn_num);
  TurnDirection get_turn_dir(int turn_dir);
  float get_run_dir(Direction dir, float ang);
  float get_turn_tgt_ang(TurnType type);
  float get_turn_rad(TurnType type);
  float get_front_dist(TurnType type, bool dia);
  float get_back_dist(TurnType type, bool dia);
  float get_slalom_etn(TurnType type, bool dia);
  float get_slalom_time(TurnType type, bool dia);
  Direction get_next_dir(Direction dir, TurnType type, TurnDirection turn_dir);
  void fix_pos(ego_odom_t &ego, TurnType type, TurnDirection turn_dir,
               trajectory_point_t &trj_ele);
  float Et2(float t, float s, float N);
  void run_straight(vector<trajectory_point_t> &trajectory, float dist,
                    trajectory_point_t &trj_ele, ego_odom_t &ego,
                    run_param_t &param, char type);
  void slalom(vector<trajectory_point_t> &trajectory, int turn_num,
              trajectory_point_t &trj_ele, ego_odom_t &ego, run_param_t &param,
              bool dia);
  float cell_size = 180;
  float half_cell_size = 90;

public:
  TrajectoryCreator(/* args */);
  ~TrajectoryCreator();
  void exec(path_struct &base_path, vector<trajectory_point_t> &trajectory);
  void exec2(path_struct &base_path, vector<trajectory_point_t> &trajectory);
  slalom_data_t sla_data;
};

#endif