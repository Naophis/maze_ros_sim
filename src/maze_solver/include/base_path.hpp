#ifndef BASE_PATH_H
#define BASE_PATH_H

#include "maze_solver.hpp"
#include "stdio.h"
#include <vector>

class BasePath {
private:
  /* data */
public:
  BasePath(/* args */);
  ~BasePath();
  void exec(path_type &path);
};

BasePath::BasePath(/* args */) {}

BasePath::~BasePath() {}

#endif