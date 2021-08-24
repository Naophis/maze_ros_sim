#ifndef MAZE_SOLVER_H
#define MAZE_SOLVER_H

#define MAX_MAZE_SIZE 32

typedef struct {
  unsigned char x;
  unsigned char y;
} point_t;

typedef struct {
  unsigned char x;
  unsigned char y;
  unsigned char dir;
  float dist2;
} dir_pt_t;

typedef struct {
  unsigned char x;
  unsigned char y;
  unsigned char dir;
  unsigned char prev_motion;
} ego_t;

typedef struct {
  float n;
  float e;
  float w;
  float s;
  unsigned int v : 8;
  unsigned int N1 : 4;
  unsigned int NE : 4;
  unsigned int E1 : 4;
  unsigned int SE : 4;
  unsigned int S1 : 4;
  unsigned int SW : 4;
  unsigned int W1 : 4;
  unsigned int NW : 4;
  unsigned int step : 4;
} vector_map_t;

enum {
  North = 1,
  East = 2,
  NorthEast = 3,
  West = 4,
  NorthWest = 5,
  SouthEast = 6,
  SouthWest = 7,
  South = 8,
} enum_Direction;

enum {
  NONE = 0,
  Straight = 1,
  Right = 2,
  Left = 3,
  Back = 4,
} enum_Motion;

enum {
  TURN_RIGHT = 1,
  TURN_LEFT = 2,
} enum_TURN_MOTION;

#endif