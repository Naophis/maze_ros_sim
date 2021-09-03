#ifndef TRAJECTORY_CREATOR_H
#define TRAJECTORY_CREATOR_H

#include "maze_solver.hpp"
#include "stdio.h"
#include <vector>
#include <cmath>

using namespace std;

class TrajectoryCreator {
private:
  /* data */
public:
  TrajectoryCreator(/* args */);
  ~TrajectoryCreator();
  void exec(path_struct &base_path, vector<trajectory_point_t> &trajectory);
  void exec2(path_struct &base_path, vector<trajectory_point_t> &trajectory);
};

#endif