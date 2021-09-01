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

void PathCreator::setNextRootDirectionPath(int x, int y, Direction now_dir,
                                           Direction dir, float &val,
                                           Direction &next_dir) {
  const bool isWall = lgc->existWall(x, y, dir);
  const bool step = lgc->isStep(x, y, dir);
  const float dist =
      isWall ? vector_max_step_val : lgc->getDistVector(x, y, dir);

  if (static_cast<int>(now_dir) * static_cast<int>(dir) == 8)
    return;
  if (!isWall && dist < val) {
    next_dir = dir;
    val = dist;
  }
}
Motion PathCreator::get_next_motion(Direction now_dir,
                                    Direction next_direction) {
  if (now_dir == next_direction)
    return Motion::Straight;

  if (now_dir == Direction::North) {
    if (next_direction == Direction::East)
      return Motion::TurnRight;
    else if (next_direction == Direction::West)
      return Motion::TurnLeft;
  } else if (now_dir == Direction::East) {
    if (next_direction == Direction::South)
      return Motion::TurnRight;
    else if (next_direction == Direction::North)
      return Motion::TurnLeft;
  } else if (now_dir == Direction::West) {
    if (next_direction == Direction::North)
      return Motion::TurnRight;
    else if (next_direction == Direction::South)
      return Motion::TurnLeft;
  } else if (now_dir == Direction::South) {
    if (next_direction == Direction::West)
      return Motion::TurnRight;
    else if (next_direction == Direction::East)
      return Motion::TurnLeft;
  }
  return Motion::Back;
}
Direction PathCreator::get_next_pos(int &x, int &y, Direction dir,
                                    Direction next_direction) {
  if (next_direction == Direction::Undefined)
    if (dir == Direction::North)
      next_direction = Direction::South;
    else if (dir == Direction::East)
      next_direction = Direction::West;
    else if (dir == Direction::West)
      next_direction = Direction::East;
    else if (dir == Direction::South)
      next_direction = Direction::North;

  if (next_direction == Direction::North)
    y++;
  else if (next_direction == Direction::East)
    x++;
  else if (next_direction == Direction::West)
    x--;
  else if (next_direction == Direction::South)
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
void PathCreator::checkOtherRoot(int x, int y, float now, Direction now_dir) {
  // int temp = 0;
  // float a[4];
  // int index = 0;
  // for (const auto d : direction_list) {
  //   float dist = lgc->getDistVector(x, y, d);
  //   if ((static_cast<int>(now_dir) * static_cast<int>(d)) != 8 &&
  //       !lgc->existWall(x, y, d) && dist < now) {
  //     temp += d;
  //     a[index] = dist;
  //     index++;
  //   }
  // }
  // float ch = a[0];
  // for (int i = 0; i < 4; i++)
  //   if (a[i] == 0)
  //     continue;
  //   else if (a[i] != ch)
  //     ch = a[i];

  // if (temp == 0)
  //   return;

  // if ((temp == 0x01) || (temp == 0x02) || (temp == 0x04) || (temp == 0x08)) {
  // } else {
  //   checkMap[x][y] = temp;
  //   addCheckQ(x, y);
  // }
}

void PathCreator::priorityStraight2(int x, int y, Direction now_dir,
                                    Direction dir, float &dist_val,
                                    Direction &next_dir) {
  const bool isWall = lgc->existWall(x, y, dir);
  const bool step = lgc->isStep(x, y, dir);
  const float dist = isWall ? MAX : lgc->getDistVector(x, y, dir);
  if (static_cast<int>(now_dir) * static_cast<int>(dir) == 8)
    return;
  if (!isWall && step && dist <= dist_val) {
    next_dir = dir;
    dist_val = dist;
  }
}
void PathCreator::clearCheckMap() {
  // for (char i = 0; i < 16; i++)
  //   for (char j = 0; j < 16; j++)
  //     checkMap[i][j] = 0;
}
void PathCreator::path_create(bool is_search) {
  Direction next_dir = Direction::North;
  Direction now_dir = next_dir;
  unsigned int idx = 0;
  Direction dirLog[3];
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
    next_dir = Direction::Undefined;

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
      setNextRootDirectionPath(x, y, now_dir, Direction::North, dist_val,
                               next_dir);
      setNextRootDirectionPath(x, y, now_dir, Direction::East, dist_val,
                               next_dir);
      setNextRootDirectionPath(x, y, now_dir, Direction::West, dist_val,
                               next_dir);
      setNextRootDirectionPath(x, y, now_dir, Direction::South, dist_val,
                               next_dir);
      if (dirLog[0] == dirLog[1] || dirLog[0] != dirLog[2]) {
        priorityStraight2(x, y, now_dir, dirLog[0], dist_val, next_dir);
      } else {
        priorityStraight2(x, y, now_dir, dirLog[1], dist_val, next_dir);
      }
    }

    Motion nextMotion = get_next_motion(now_dir, next_dir);

    // ROS_WARN("%d, %d, %d", position, now_dir, next_dir);
    // checkOtherRoot(x, y, position, now_dir);

    if (nextMotion == Motion::Straight) {
      add_path_s(idx, 2);
    } else if (nextMotion == Motion::TurnRight) {
      path_t.push_back(static_cast<int>(TurnDirection::Right));
      path_s.push_back(2);
      idx++;
    } else if (nextMotion == Motion::TurnLeft) {
      path_t.push_back(static_cast<int>(TurnDirection::Left));
      path_s.push_back(2);
      idx++;
    } else {
      path_t.push_back(255);
      break;
    }
    next_dir = get_next_pos(x, y, now_dir, next_dir);
  }
}