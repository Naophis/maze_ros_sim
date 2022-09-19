#ifndef PATH_CREATOR_H
#define PATH_CREATOR_H

#include "stdio.h"
#include <ros/ros.h>
#include <vector>

#include "logic.hpp"
#include "maze_solver.hpp"
#include <queue>

using namespace std;

constexpr int checkQlength = 256;
constexpr int R = 1;
constexpr int L = 2;
class PathCreator {
private:
  int get_dist_val(int x, int y);
  void updateVectorMap(const bool isSearch);
  unsigned int vector_max_step_val = 180 * 1024;
  float MAX = vector_max_step_val;

  void add_path_s(int idx, float val);
  void append_path_s(float val);
  void append_path_t(int val);
  void setNextRootDirectionPath(int x, int y, Direction now_dir, Direction dir,
                                float &val, Direction &next_dir);
  Direction get_next_pos(int &x, int &y, Direction dir,
                         Direction next_direction);

  Motion get_next_motion(Direction now_dir, Direction next_direction);

  void priorityStraight2(int x, int y, Direction now_dir, Direction dir,
                         float &dist_val, Direction &next_dir);

  void checkOtherRoot(int x, int y, float now, Direction now_dir);

  void clearCheckMap();
  void addCheckQ(int x, int y);

  unsigned char checkMap[16][16];

  int checkQ[checkQlength];

  void pathOffset();

public:
  MazeSolverBaseLgc *lgc;

  vector<float> path_s;
  vector<int> path_t;
  int path_size;

  PathCreator(/* args */);
  ~PathCreator();
  void path_create(bool is_search);
  void path_reflash();
  void convert_large_path(bool b1);
  void diagonalPath(bool isFull, bool a1);
  float drawChangePathRoot(char goalX, char goalY, char isFull);
};

#endif