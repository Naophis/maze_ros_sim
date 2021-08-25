#include "path_creator.hpp"

PathCreator::PathCreator(/* args */) {}

PathCreator::~PathCreator() {}

int PathCreator::get_dist_val(int x, int y) {
  //
  return lgc->get_dist_val(x, y);
}

void PathCreator::updateVectorMap(const bool isSearch) {
  lgc->updateVectorMap(isSearch);
}

void PathCreator::path_reflash() {
  path_s.clear();
  path_t.clear();
}

void PathCreator::append_path_s(float val) {
  path_s.push_back(val); //
}
void PathCreator::append_path_t(int val) {
  path_t.push_back(val); //
}
void PathCreator::add_path_s(int idx, float val) {
  path_s[idx] += val; //
}

void PathCreator::setNextRootDirectionPath(int x, int y, int now_dir, int dir,
                                           float &val, int &next_dir) {
  bool isWall = lgc->existWall(x, y, dir);
  bool step = lgc->isStep(x, y, dir);
  float dist = isWall ? vector_max_step_val : lgc->getDistVector(x, y, dir);

  if (now_dir * dir == 8)
    return;
  if (!isWall && dist < val) {
    next_dir = dir;
    val = dist;
  }
}
int PathCreator::get_next_motion(int now_dir, int next_direction) {
  if (now_dir == next_direction)
    return Straight;

  if (now_dir == North) {
    if (next_direction == East)
      return Right;
    else if (next_direction == West)
      return Left;
  } else if (now_dir == East) {
    if (next_direction == South)
      return Right;
    else if (next_direction == North)
      return Left;
  } else if (now_dir == West) {
    if (next_direction == North)
      return Right;
    else if (next_direction == South)
      return Left;
  } else if (now_dir == South) {
    if (next_direction == West)
      return Right;
    else if (next_direction == East)
      return Left;
  }
  return Back;
}
int PathCreator::get_next_pos(int &x, int &y, int dir, int next_direction) {
  if (next_direction == 255)
    if (dir == North)
      next_direction = South;
    else if (dir == East)
      next_direction = West;
    else if (dir == West)
      next_direction = East;
    else if (dir == South)
      next_direction = North;

  if (next_direction == North)
    y++;
  else if (next_direction == East)
    x++;
  else if (next_direction == West)
    x--;
  else if (next_direction == South)
    y--;

  return next_direction;
}

void PathCreator::addCheckQ(int x, int y) {
  for (int i = 0; i < checkQlength; i++) {
    if ((checkQ[i] & 0xf0) >> 4 == x && (checkQ[i] & 0x0f) == y) {
      return;
    }
  }
  for (int i = 0; i < checkQlength; i++) {
    if (checkQ[i] == 0) {
      checkQ[i] = x * 16 + y;
      return;
    }
  }
}
void PathCreator::checkOtherRoot(int x, int y, float now, int now_dir) {
  int temp = 0;
  float a[4];
  int index = 0;
  for (int d = 1; d <= 8; d *= 2) {
    float dist = lgc->getDistVector(x, y, d);
    if (now_dir * d != 8 && !lgc->existWall(x, y, d) && dist < now) {
      temp += d;
      a[index] = dist;
      index++;
    }
  }
  float ch = a[0];
  for (int i = 0; i < 4; i++)
    if (a[i] == 0)
      continue;
    else if (a[i] != ch)
      ch = a[i];

  if (temp == 0)
    return;

  if ((temp == 0x01) || (temp == 0x02) || (temp == 0x04) || (temp == 0x08)) {
  } else {
    checkMap[x][y] = temp;
    addCheckQ(x, y);
  }
}

void PathCreator::priorityStraight2(int x, int y, int now_dir, int dir,
                                    float &dist_val, int &next_dir) {
  char isWall = lgc->existWall(x, y, dir);
  char step = lgc->isStep(x, y, dir);
  float dist = isWall ? MAX : lgc->getDistVector(x, y, dir);
  if (now_dir * dir == 8)
    return;
  if (!isWall && step && dist <= dist_val) {
    next_dir = dir;
    dist_val = dist;
  }
}
void PathCreator::clearCheckMap() {
  for (char i = 0; i < 16; i++)
    for (char j = 0; j < 16; j++)
      checkMap[i][j] = 0;
}
void PathCreator::path_create(bool is_search) {
  int next_dir = North;
  int now_dir = next_dir;
  int nextMotion = 0;
  unsigned int idx = 0;
  int dirLog[3];
  bool b1 = false;
  bool b2 = false;

  int x = 0;
  int y = 1;

  path_reflash();
  clearCheckMap();
  lgc->updateVectorMap(is_search);

  path_s.push_back(1);
  float dist_val;
  while (true) {
    now_dir = next_dir;
    dirLog[2] = dirLog[1];
    dirLog[1] = dirLog[0];
    dirLog[0] = now_dir;

    dist_val = MAX;
    next_dir = 255;

    if (lgc->arrival_goal_position(x, y)) {
      path_t.push_back(255);
      return;
    }
    const float position = lgc->getDistVector(x, y, now_dir);

    if (b1) {
      // if (dirLog[0] == dirLog[1] || dirLog[0] != dirLog[2]) {
      //   if (!setNextRootDirection(x, y, dirLog[0])) {
      //     break;
      //   }
      // } else {
      //   if (!setNextRootDirection(x, y, dirLog[1])) {
      //     if (!b2) {
      //       priorityStraight2(x, y, dirLog[0]);
      //     } else {
      //       break;
      //     }
      //   }
      //   b2 = true;
      // }
    } else {
      setNextRootDirectionPath(x, y, now_dir, North, dist_val, next_dir);
      setNextRootDirectionPath(x, y, now_dir, East, dist_val, next_dir);
      setNextRootDirectionPath(x, y, now_dir, West, dist_val, next_dir);
      setNextRootDirectionPath(x, y, now_dir, South, dist_val, next_dir);
      if (dirLog[0] == dirLog[1] || dirLog[0] != dirLog[2]) {
        priorityStraight2(x, y, now_dir, dirLog[0], dist_val, next_dir);
      } else {
        priorityStraight2(x, y, now_dir, dirLog[1], dist_val, next_dir);
      }
    }

    int nextMotion = get_next_motion(now_dir, next_dir);

    // ROS_WARN("%d, %d, %d", position, now_dir, next_dir);
    // checkOtherRoot(x, y, position, now_dir);

    if (nextMotion == Straight) {
      add_path_s(idx, 2);
    } else if (nextMotion == Right) {
      path_t.push_back(TURN_RIGHT);
      path_s.push_back(2);
      idx++;
    } else if (nextMotion == Left) {
      path_t.push_back(TURN_LEFT);
      path_s.push_back(2);
      idx++;
    } else {
      path_t.push_back(255);
      break;
    }
    next_dir = get_next_pos(x, y, now_dir, next_dir);
  }
}