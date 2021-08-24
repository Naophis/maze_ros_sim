#ifndef PATH_CREATOR_H
#define PATH_CREATOR_H

#include "stdio.h"
#include <ros/ros.h>
#include <vector>

#include "logic.hpp"
#include "maze_solver.hpp"

using namespace std;

#define checkQlength 256
class PathCreator {
private:
  int get_dist_val(int x, int y);
  void updateVectorMap(const bool isSearch);
  unsigned int vector_max_step_val = 180 * 1024;
  float MAX = vector_max_step_val;

  void add_path_s(int idx, float val);
  void append_path_s(float val);
  void append_path_t(int val);
  void setNextRootDirectionPath(int x, int y, int now_dir, int dir, float &val,
                                int &next_dir);
  int get_next_pos(int &x, int &y, int dir, int next_direction);

  int get_next_motion(int now_dir, int next_direction);

  void priorityStraight2(int x, int y, int now_dir, int dir, float &dist_val,
                         int &next_dir);

  void checkOtherRoot(int x, int y, float now, int now_dir);

  void clearCheckMap();
  void addCheckQ(int x, int y);

  unsigned char checkMap[16][16];

  int checkQ[checkQlength];

public:
  MazeSolverBaseLgc *lgc;

  vector<float> path_s;
  vector<int> path_t;
  PathCreator(/* args */);
  ~PathCreator();
  void path_create(bool is_search);
  void path_reflash();
};

#endif