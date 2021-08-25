#include <ros/ros.h>

#include "logic.hpp"

void MazeSolverBaseLgc::init(const int _maze_size, const int _max_step_val) {
  maze_size = _maze_size;
  max_step_val = _max_step_val;
  maze_list_size = maze_size * maze_size;

  map.resize(maze_list_size);
  dist.resize(maze_list_size);
  vector_dist.resize(maze_list_size);
  updateMap.resize(maze_list_size);

  q_list.resize(maze_list_size + 1);
  vq_list.resize(4 * maze_list_size + 1);
  goal_list3.clear();
  point_t p;
  p.x = maze_size - 1;
  p.y = 0;
  goal_list3.push_back(p);
  p.x = maze_size - 1;
  p.y = maze_size - 1;
  goal_list3.push_back(p);
  p.x = 0;
  p.y = maze_size - 1;
  goal_list3.push_back(p);
  p.x = 0;
  p.y = maze_size - 1;
  goal_list3.push_back(p);
}

void MazeSolverBaseLgc::set_goal_pos(const vector<point_t> &list) {
  goal_list.clear();
  for (const auto p : list)
    goal_list.push_back(p);
}

void MazeSolverBaseLgc::set_goal_pos2(const vector<point_t> &pt_list) {
  goal_list2.clear();
  for (const auto p : pt_list)
    goal_list2.push_back(p);
}

bool MazeSolverBaseLgc::valid_map_list_idx(const int x, const int y) {
  if (x < 0 || x >= maze_size || y < 0 || y >= maze_size)
    return false;

  if ((x + y * maze_size) >= maze_list_size)
    return false;

  return true;
}

bool MazeSolverBaseLgc::candidate_end(const int x, const int y) {
  unsigned char temp = map[x + y * maze_size] & 0x0f;
  return (temp == 0x0e || temp == 0x0d || temp == 0x0b || temp == 0x07 ||
          temp == 0x0f);
}
void MazeSolverBaseLgc::updateWall(int x, int y, int dir) {
  if (dir == North) {
    set_wall_data(x, y, North, true);
    set_wall_data(x, y + 1, South, true);
  } else if (dir == East) {
    set_wall_data(x, y, East, true);
    set_wall_data(x + 1, y, West, true);
  } else if (dir == West) {
    set_wall_data(x, y, West, true);
    set_wall_data(x - 1, y, East, true);
  } else if (dir == South) {
    set_wall_data(x, y, South, true);
    set_wall_data(x, y - 1, North, true);
  }
}

void MazeSolverBaseLgc::remove_goal_pos3() {
  for (auto it = goal_list3.begin(); it != goal_list3.end();) {
    bool flag1 = false;
    flag1 |= is_stepped((*it).x, (*it).y);
    flag1 |= get_dist_val((*it).x, (*it).y) == max_step_val;

    if (flag1)
      it = goal_list3.erase(it);
    else
      it++;
  }
}

void MazeSolverBaseLgc::update_dist_map(const int mode,
                                        const bool search_mode) {
  int c = 0;
  for (int i = 0; i < maze_size; i++)
    for (int j = 0; j < maze_size; j++)
      dist[c++] = max_step_val;

  int head = 0;
  int tail = 0;
  if (!search_mode) {
    for (const auto g : goal_list) {
      int idx = g.x + g.y * maze_size;
      dist[idx] = 0;
      q_list[tail].x = g.x;
      q_list[tail].y = g.y;
      tail++;
    }
    // for (const auto g : goal_list3) {
    //   int idx = g.x + g.y * maze_size;
    //   dist[idx] = 0;
    //   q_list[tail].x = g.x;
    //   q_list[tail].y = g.y;
    //   tail++;
    // }
  } else {
    for (const auto g : goal_list2) {
      int idx = g.x + g.y * maze_size;
      dist[idx] = 0;
      q_list[tail].x = g.x;
      q_list[tail].y = g.y;
      tail++;
    }
    for (const auto g : goal_list3) {
      int idx = g.x + g.y * maze_size;
      dist[idx] = 0;
      q_list[tail].x = g.x;
      q_list[tail].y = g.y;
      tail++;
    }
  }
  char D = 1;
  int pt1;
  int b;
  char X = 0, Y = 0;
  char i, j;
  while (head != tail) {
    Y = q_list[head].y;
    X = q_list[head].x;
    head++;
    pt1 = get_dist_val(X, Y) + 1;
    D = 1;
    while (D <= 8) {
      i = 0;
      j = 0;
      if (D == North)
        j = 1;
      else if (D == East)
        i = 1;
      else if (D == West)
        i = -1;
      else if (D == South)
        j = -1;

      b = false;
      if (mode == 1)
        b = isProceed(X, Y, D);
      else
        b = !existWall(X, Y, D);

      if (b && get_dist_val(X + i, Y + j) == max_step_val) {
        if (X + i < 0 || X + i >= maze_size || Y + j < 0 ||
            Y + j >= maze_size) {
        } else {
          set_dist_val(X + i, Y + j, pt1);
          q_list[tail].x = X + i;
          q_list[tail].y = Y + j;
          tail++;
        }
      }
      D *= 2;
    }
  }
}

void MazeSolverBaseLgc::set_dist_val(const int x, const int y, const int val) {
  if (valid_map_list_idx(x, y))
    dist[x + y * maze_size] = val;
}

int MazeSolverBaseLgc::get_dist_val(int x, int y) {
  if (valid_map_list_idx(x, y))
    return dist[x + y * maze_size];
  else
    return max_step_val;
}

float MazeSolverBaseLgc::get_diadist_n_val(const int x, const int y) {
  if (valid_map_list_idx(x, y))
    return vector_dist[x + y * maze_size].n;
  else
    return vector_max_step_val;
}

float MazeSolverBaseLgc::get_diadist_e_val(const int x, const int y) {
  if (valid_map_list_idx(x, y))
    return vector_dist[x + y * maze_size].e;
  else
    return vector_max_step_val;
}

int MazeSolverBaseLgc::get_map_val(const int x, const int y) {
  if (valid_map_list_idx(x, y))
    return map[x + y * maze_size];
  else
    return 0xff;
}

bool MazeSolverBaseLgc::isProceed(const int x, const int y, const int dir) {
  if (valid_map_list_idx(x, y))
    return ((get_map_val(x, y) / dir) & 0x11) == 0x10;
  else
    return false;
}

bool MazeSolverBaseLgc::existWall(const int x, const int y, const int dir) {
  if (valid_map_list_idx(x, y))
    return ((get_map_val(x, y) / dir) & 0x01) == 0x01;
  else
    return true;
}

void MazeSolverBaseLgc::set_map_val(const int x, const int y, const int val) {
  if (valid_map_list_idx(x, y))
    map[x + y * maze_size] = val;
}
void MazeSolverBaseLgc::set_map_val(int idx, int val) { map[idx] = val; }

void MazeSolverBaseLgc::set_wall_data(const int x, const int y, const int dir,
                                      const bool isWall) {
  if (valid_map_list_idx(x, y)) {
    int idx = x + y * maze_size;
    map[idx] |= (0x10 * dir);
    if (isWall)
      map[idx] |= 0x01 * dir;
    else
      map[idx] = (map[idx] & 0xf0) | (map[idx] & (~(0x01 * dir) & 0x0f));
  }
}

void MazeSolverBaseLgc::set_default_wall_data() {
  for (int i = 0; i < maze_size; i++) {
    // set_map
    set_wall_data(i, maze_size - 1, North, true);
    set_wall_data(maze_size - 1, i, East, true);
    set_wall_data(0, i, West, true);
    set_wall_data(i, 0, South, true);
  }
  set_wall_data(0, 0, East, true);
  set_wall_data(0, 0, North, false);
  set_wall_data(1, 0, West, true);
  set_wall_data(0, 1, South, false);
}

bool MazeSolverBaseLgc::isStep(const int x, const int y, const int dir) {
  if (valid_map_list_idx(x, y))
    return ((map[x + y * maze_size] / dir) & 0x10) == 0x10;
  return false;
}

void MazeSolverBaseLgc::back_home() {
  clear_goal();
  point_t p;
  p.x = p.y = 0;
  goal_list.push_back(p);
}

void MazeSolverBaseLgc::clear_goal() { goal_list.clear(); }

void MazeSolverBaseLgc::append_goal(const int x, const int y) {
  point_t p;
  p.x = p.y = 0;
  goal_list.push_back(p);
}

int MazeSolverBaseLgc::get_max_step_val() { return max_step_val; }

int MazeSolverBaseLgc::clear_vector_distmap() {
  int tail = 0;
  for (char i = 0; i < maze_size; i++) {
    for (char j = 0; j < maze_size; j++) {
      int idx = i + j * maze_size;
      vector_dist[idx].n = vector_max_step_val;
      vector_dist[idx].e = vector_max_step_val;
      vector_dist[idx].w = vector_max_step_val;
      vector_dist[idx].s = vector_max_step_val;
      vector_dist[idx].step = 0;
      vector_dist[idx].v = 0;
      vector_dist[idx].N1 = 0;
      vector_dist[idx].E1 = 0;
      vector_dist[idx].W1 = 0;
      vector_dist[idx].NE = 0;
      vector_dist[idx].NW = 0;
      vector_dist[idx].SE = 0;
      vector_dist[idx].SW = 0;
      updateMap[idx] = 0;
    }
  }
  int vq_list_size = vq_list.size();
  for (int i = 0; i < vq_list_size; i++)
    vq_list[i].x = vq_list[i].y = vq_list[i].dir = vq_list[i].dist2 = 0;

  for (const auto p : goal_list) {
    unsigned char x = p.x;
    unsigned char y = p.y;
    int idx = x + y * maze_size;
    if (!existWall(x, y, North)) {
      vector_dist[idx].n = 0;
      vq_list[tail].x = x;
      vq_list[tail].y = y;
      vq_list[tail].dir = North;
      tail++;
      if (y < maze_size - 1)
        vector_dist[(x) + (y + 1) * maze_size].s = 0;
    }
    if (!existWall(x, y, East)) {
      vector_dist[idx].e = 0;
      vq_list[tail].x = x;
      vq_list[tail].y = y;
      vq_list[tail].dir = East;
      tail++;
      if (x < maze_size - 1)
        vector_dist[(x + 1) + (y)*maze_size].w = 0;
    }
    if (!existWall(x, y, West)) {
      vector_dist[idx].w = 0;
      vq_list[tail].x = x;
      vq_list[tail].y = y;
      vq_list[tail].dir = West;
      tail++;
      if (x > 0)
        vector_dist[(x - 1) + (y)*maze_size].e = 0;
    }
    if (!existWall(x, y, South)) {
      vector_dist[idx].s = 0;
      vq_list[tail].x = x;
      vq_list[tail].y = y;
      vq_list[tail].dir = South;
      tail++;
      if (y > 0)
        vector_dist[(x) + (y - 1) * maze_size].n = 0;
    }
  }

  return tail;
}

int MazeSolverBaseLgc::clear_vector_distmap(
    unordered_map<unsigned int, unsigned char> &subgoal_list) {
  int tail = 0;
  for (char i = 0; i < maze_size; i++) {
    for (char j = 0; j < maze_size; j++) {
      int idx = i + j * maze_size;
      if (vector_dist[idx].n == vector_max_step_val)
        subgoal_list.erase(idx);
      vector_dist[idx].n = vector_max_step_val;
      vector_dist[idx].e = vector_max_step_val;
      vector_dist[idx].w = vector_max_step_val;
      vector_dist[idx].s = vector_max_step_val;
      vector_dist[idx].step = 0;
      vector_dist[idx].v = 0;
      vector_dist[idx].N1 = 0;
      vector_dist[idx].E1 = 0;
      vector_dist[idx].W1 = 0;
      vector_dist[idx].NE = 0;
      vector_dist[idx].NW = 0;
      vector_dist[idx].SE = 0;
      vector_dist[idx].SW = 0;
      updateMap[idx] = 0;
    }
  }
  int vq_list_size = vq_list.size();
  for (int i = 0; i < vq_list_size; i++)
    vq_list[i].x = vq_list[i].y = vq_list[i].dir = vq_list[i].dist2 = 0;

  for (const auto p : goal_list) {
    unsigned char x = p.x;
    unsigned char y = p.y;
    int idx = x + y * maze_size;
    if (!existWall(x, y, North)) {
      vector_dist[idx].n = 0;
      vq_list[tail].x = x;
      vq_list[tail].y = y;
      vq_list[tail].dir = North;
      tail++;
      if (y < maze_size - 1)
        vector_dist[(x) + (y + 1) * maze_size].s = 0;
    }
    if (!existWall(x, y, East)) {
      vector_dist[idx].e = 0;
      vq_list[tail].x = x;
      vq_list[tail].y = y;
      vq_list[tail].dir = East;
      tail++;
      if (x < maze_size - 1)
        vector_dist[(x + 1) + (y)*maze_size].w = 0;
    }
    if (!existWall(x, y, West)) {
      vector_dist[idx].w = 0;
      vq_list[tail].x = x;
      vq_list[tail].y = y;
      vq_list[tail].dir = West;
      tail++;
      if (x > 0)
        vector_dist[(x - 1) + (y)*maze_size].e = 0;
    }
    if (!existWall(x, y, South)) {
      vector_dist[idx].s = 0;
      vq_list[tail].x = x;
      vq_list[tail].y = y;
      vq_list[tail].dir = South;
      tail++;
      if (y > 0)
        vector_dist[(x) + (y - 1) * maze_size].n = 0;
    }
  }

  return tail;
}

int MazeSolverBaseLgc::haveVectorLv(const int x, const int y, const int dir) {
  const int idx = x + y * maze_size;
  if (dir == North) {
    if (vector_dist[idx].N1 > borderLv2)
      return 2;
    else if (vector_dist[idx].N1 > borderLv1)
      return 1;
  } else if (dir == NorthEast) {
    if (vector_dist[idx].NE > borderLv2d)
      return 2;
    else if (vector_dist[idx].NE > borderLv1d)
      return 1;
  } else if (dir == East) {
    if (vector_dist[idx].E1 > borderLv2)
      return 2;
    else if (vector_dist[idx].E1 > borderLv1)
      return 1;
  } else if (dir == SouthEast) {
    if (vector_dist[idx].SE > borderLv2d)
      return 2;
    else if (vector_dist[idx].SE > borderLv1d)
      return 1;
  } else if (dir == South) {
    if (vector_dist[idx].S1 > borderLv2)
      return 2;
    else if (vector_dist[idx].S1 > borderLv1)
      return 1;
  } else if (dir == SouthWest) {
    if (vector_dist[idx].SW > borderLv2d)
      return 2;
    else if (vector_dist[idx].SW > borderLv1d)
      return 1;
  } else if (dir == West) {
    if (vector_dist[idx].W1 > borderLv2)
      return 2;
    else if (vector_dist[idx].W1 > borderLv1)
      return 1;
  } else if (dir == NorthWest) {
    if (vector_dist[idx].NW > borderLv2d)
      return 2;
    else if (vector_dist[idx].NW > borderLv1d)
      return 1;
  }
  return 0;
}

float MazeSolverBaseLgc::getDistV(const int x, const int y, const int dir) {
  if (valid_map_list_idx(x, y)) {
    if (dir == North)
      return vector_dist[x + y * maze_size].n;
    else if (dir == East)
      return vector_dist[x + y * maze_size].e;
    else if (dir == West)
      return vector_dist[x + y * maze_size].w;
    else if (dir == South)
      return vector_dist[x + y * maze_size].s;
  }
  return vector_max_step_val;
}

void MazeSolverBaseLgc::setDistV(const int x, const int y, const int dir,
                                 const float val) {
  if (valid_map_list_idx(x, y)) {
    const int idx = x + y * maze_size;
    if (dir == North) {
      vector_dist[idx].n = val;
      if (y < maze_size - 1)
        vector_dist[(x) + (y + 1) * maze_size].s = val;
    } else if (dir == East) {
      vector_dist[idx].e = val;
      if (x < maze_size - 1)
        vector_dist[(x + 1) + (y)*maze_size].w = val;
    } else if (dir == West) {
      vector_dist[idx].w = val;
      if (x > 0)
        vector_dist[(x - 1) + (y)*maze_size].e = val;
    } else if (dir == South) {
      vector_dist[idx].s = val;
      if (y > 0)
        vector_dist[(x) + (y - 1) * maze_size].n = val;
    }
  }
}

bool MazeSolverBaseLgc::isUpdated(const int x, const int y, const int dir) {
  if (!valid_map_list_idx(x, y))
    return true;

  const int idx = x + y * maze_size;
  if (dir == North)
    return (updateMap[idx] & 0x01) == 0x01;
  else if (dir == East)
    return (updateMap[idx] & 0x02) == 0x02;
  else if (dir == West)
    return (updateMap[idx] & 0x04) == 0x04;
  else if (dir == South)
    return (updateMap[idx] & 0x08) == 0x08;
  else if (dir == NorthEast)
    return (updateMap[idx] & 0x10) == 0x10;
  else if (dir == SouthEast)
    return (updateMap[idx] & 0x20) == 0x20;
  else if (dir == SouthWest)
    return (updateMap[idx] & 0x40) == 0x40;
  else if (dir == NorthWest)
    return (updateMap[idx] & 0x80) == 0x80;

  return false;
}

void MazeSolverBaseLgc::addVector(const int x, const int y, const int dir,
                                  float val) {
  if (valid_map_list_idx(x, y)) {
    if (val < 15)
      val++;
    const int idx = x + y * maze_size;
    if (dir == North)
      vector_dist[idx].N1 = val;
    else if (dir == NorthEast)
      vector_dist[idx].NE = val;
    else if (dir == East)
      vector_dist[idx].E1 = val;
    else if (dir == SouthEast)
      vector_dist[idx].SE = val;
    else if (dir == South)
      vector_dist[idx].S1 = val;
    else if (dir == SouthWest)
      vector_dist[idx].SW = val;
    else if (dir == West)
      vector_dist[idx].W1 = val;
    else if (dir == NorthWest)
      vector_dist[idx].NW = val;
  }
}

void MazeSolverBaseLgc::updateMapCheck(const int x, const int y,
                                       const int dir) {
  if (valid_map_list_idx(x, y)) {
    const int idx = x + y * maze_size;
    if (dir == North)
      updateMap[idx] |= 0x01;
    else if (dir == East)
      updateMap[idx] |= 0x02;
    else if (dir == West)
      updateMap[idx] |= 0x04;
    else if (dir == South)
      updateMap[idx] |= 0x08;
    else if (dir == NorthEast)
      updateMap[idx] |= 0x10;
    else if (dir == SouthEast)
      updateMap[idx] |= 0x20;
    else if (dir == SouthWest)
      updateMap[idx] |= 0x40;
    else if (dir == NorthWest)
      updateMap[idx] |= 0x80;
  }
}

int MazeSolverBaseLgc::getVector(const int x, const int y, const int dir) {
  if (valid_map_list_idx(x, y)) {
    const int idx = x + y * maze_size;
    if (dir == North)
      return vector_dist[idx].N1;
    else if (dir == NorthEast)
      return vector_dist[idx].NE;
    else if (dir == East)
      return vector_dist[idx].E1;
    else if (dir == SouthEast)
      return vector_dist[idx].SE;
    else if (dir == South)
      return vector_dist[idx].S1;
    else if (dir == SouthWest)
      return vector_dist[idx].SW;
    else if (dir == West)
      return vector_dist[idx].W1;
    else if (dir == NorthWest)
      return vector_dist[idx].NW;
  }
  return 0;
}

bool MazeSolverBaseLgc::is_unknown(const int x, const int y, const int dir) {
  if (valid_map_list_idx(x, y)) {
    return (map[x + y * maze_size] & (0x10 * dir)) == 0x00;
  }
  return false;
}

void MazeSolverBaseLgc::simplesort(const int tail) {
  //  sort(&vq_list[0], &vq_list[tail],
  //       [](const auto &a, const auto &b) { return a.dist2 < b.dist2; });
  float temp[4];
  for (int i = tail; i >= 0; i--) {
    if (vq_list[i].dist2 < vq_list[i - 1].dist2) {
      temp[0] = vq_list[i].x;
      temp[1] = vq_list[i].y;
      temp[2] = vq_list[i].dir;
      temp[3] = vq_list[i].dist2;
      vq_list[i].x = vq_list[i - 1].x;
      vq_list[i].y = vq_list[i - 1].y;
      vq_list[i].dir = vq_list[i - 1].dir;
      vq_list[i].dist2 = vq_list[i - 1].dist2;
      vq_list[i - 1].x = temp[0];
      vq_list[i - 1].y = temp[1];
      vq_list[i - 1].dir = temp[2];
      vq_list[i - 1].dist2 = temp[3];
    } else {
      break;
    }
  }
}

unsigned int MazeSolverBaseLgc::updateVectorMap(const bool isSearch) {
  unsigned int head = 0;
  unsigned int tail = clear_vector_distmap();
  unsigned int c = 0;

  while (head != tail) {
    int X = vq_list[head].x;
    int Y = vq_list[head].y;
    int dir = vq_list[head].dir;
    int i = 0;
    int j = 0;
    int d[3];
    int d2[3];
    float now = getDistV(X, Y, dir);
    if (dir == North) {
      j = 1;
      d[0] = North;     // N
      d[1] = NorthEast; // NE
      d[2] = NorthWest; // NW
      d2[0] = North;    // N
      d2[1] = East;     // E
      d2[2] = West;     // W
    } else if (dir == East) {
      i = 1;
      d[0] = East;      // E
      d[1] = SouthEast; // SE
      d[2] = NorthEast; // NE
      d2[0] = East;     // E
      d2[1] = South;    // S
      d2[2] = North;    // N
    } else if (dir == West) {
      i = -1;
      d[0] = West;      // W
      d[1] = NorthWest; // NW
      d[2] = SouthWest; // SW
      d2[0] = West;     // W
      d2[1] = North;    // N
      d2[2] = South;    // S
    } else if (dir == South) {
      j = -1;
      d[0] = South;     // S
      d[1] = SouthWest; // SW
      d[2] = SouthEast; // SE
      d2[0] = South;    // S
      d2[1] = West;     // W
      d2[2] = East;     // E
    }
    // c++;
    for (int k = 0; k < 3; k++) {
      c++;

      if (!existWall(X + i, Y + j, d2[k]) &&
          (isSearch || isStep(X + i, Y + j, d2[k]))) {
        int v = haveVectorLv(X, Y, d[k]);
        float tmp = now;
        if (dir == d2[k]) {
          if (v >= 2) {
            tmp += St3;
          } else if (v == 1) {
            tmp += St2;
          } else {
            tmp += St1;
          }
          if (tmp <= getDistV(X + i, Y + j, d2[k])) {
            if (!isUpdated(X + i, Y + j, d2[k])) {
              setDistV(X + i, Y + j, d2[k], tmp);
              vq_list[tail].x = (X + i);
              vq_list[tail].y = (Y + j);
              vq_list[tail].dir = d2[k];
              vq_list[tail].dist2 = tmp;
              simplesort(tail);
              tail++;
              updateMapCheck(X + i, Y + j, d2[k]);
            }
          }
          addVector(X + i, Y + j, d[k], getVector(X, Y, d[k]));
        } else {
          if (v == 2) {
            tmp += Dia3;
          } else if (v == 1) {
            tmp += Dia2;
          } else {
            tmp += Dia;
          }
          if (tmp <= getDistV(X + i, Y + j, d2[k])) {
            if (!isUpdated(X + i, Y + j, d2[k])) {
              setDistV(X + i, Y + j, d2[k], tmp);
              vq_list[tail].x = (X + i);
              vq_list[tail].y = (Y + j);
              vq_list[tail].dir = d2[k];
              vq_list[tail].dist2 = tmp;
              simplesort(tail);
              tail++;
              updateMapCheck(X + i, Y + j, d2[k]);
            }
          }
          addVector(X + i, Y + j, d[k], getVector(X, Y, d[k]));
        }
      }
    }
    head++;
  }
  return c;
}

unsigned int MazeSolverBaseLgc::updateVectorMap(
    const bool isSearch,
    unordered_map<unsigned int, unsigned char> &subgoal_list) {
  unsigned int head = 0;
  unsigned int tail = clear_vector_distmap(subgoal_list);
  unsigned int c = 0;

  while (head != tail) {
    int X = vq_list[head].x;
    int Y = vq_list[head].y;

    if ((map[X + Y * maze_size] & 0xf0) == 0xf0) {
      subgoal_list.erase(X + Y * maze_size);
    }

    int dir = vq_list[head].dir;
    int i = 0;
    int j = 0;
    int d[3];
    int d2[3];
    float now = getDistV(X, Y, dir);
    if (dir == North) {
      j = 1;
      d[0] = North;     // N
      d[1] = NorthEast; // NE
      d[2] = NorthWest; // NW
      d2[0] = North;    // N
      d2[1] = East;     // E
      d2[2] = West;     // W
    } else if (dir == East) {
      i = 1;
      d[0] = East;      // E
      d[1] = SouthEast; // SE
      d[2] = NorthEast; // NE
      d2[0] = East;     // E
      d2[1] = South;    // S
      d2[2] = North;    // N
    } else if (dir == West) {
      i = -1;
      d[0] = West;      // W
      d[1] = NorthWest; // NW
      d[2] = SouthWest; // SW
      d2[0] = West;     // W
      d2[1] = North;    // N
      d2[2] = South;    // S
    } else if (dir == South) {
      j = -1;
      d[0] = South;     // S
      d[1] = SouthWest; // SW
      d[2] = SouthEast; // SE
      d2[0] = South;    // S
      d2[1] = West;     // W
      d2[2] = East;     // E
    }
    // c++;
    for (int k = 0; k < 3; k++) {
      c++;

      if (!existWall(X + i, Y + j, d2[k]) &&
          (isSearch || isStep(X + i, Y + j, d2[k]))) {
        int v = haveVectorLv(X, Y, d[k]);
        float tmp = now;
        if (dir == d2[k]) {
          if (v >= 2) {
            tmp += St3;
          } else if (v == 1) {
            tmp += St2;
          } else {
            tmp += St1;
          }
          if (tmp <= getDistV(X + i, Y + j, d2[k])) {
            if (!isUpdated(X + i, Y + j, d2[k])) {
              setDistV(X + i, Y + j, d2[k], tmp);
              vq_list[tail].x = (X + i);
              vq_list[tail].y = (Y + j);
              vq_list[tail].dir = d2[k];
              vq_list[tail].dist2 = tmp;
              simplesort(tail);
              tail++;
              updateMapCheck(X + i, Y + j, d2[k]);
            }
          }
          addVector(X + i, Y + j, d[k], getVector(X, Y, d[k]));
        } else {
          if (v == 2) {
            tmp += Dia3;
          } else if (v == 1) {
            tmp += Dia2;
          } else {
            tmp += Dia;
          }
          if (tmp <= getDistV(X + i, Y + j, d2[k])) {
            if (!isUpdated(X + i, Y + j, d2[k])) {
              setDistV(X + i, Y + j, d2[k], tmp);
              vq_list[tail].x = (X + i);
              vq_list[tail].y = (Y + j);
              vq_list[tail].dir = d2[k];
              vq_list[tail].dist2 = tmp;
              simplesort(tail);
              tail++;
              updateMapCheck(X + i, Y + j, d2[k]);
            }
          }
          addVector(X + i, Y + j, d[k], getVector(X, Y, d[k]));
        }
      }
    }
    head++;
  }
  return c;
}

bool MazeSolverBaseLgc::is_stepped(int x, int y) {
  if (valid_map_list_idx(x, y))
    return ((map[x + y * maze_size]) & 0xf0) == 0xf0;
  return false;
}

float MazeSolverBaseLgc::getDistVector(const int x, const int y,
                                       const int dir) {
  if (valid_map_list_idx(x, y)) {
    if (existWall(x, y, dir))
      return VectorMax;
    if (dir == North)
      return vector_dist[x + y * maze_size].n;
    else if (dir == East)
      return vector_dist[x + y * maze_size].e;
    else if (dir == West)
      return vector_dist[x + y * maze_size].w;
    else if (dir == South)
      return vector_dist[x + y * maze_size].s;
  }
  return VectorMax;
}

void MazeSolverBaseLgc::setNextRootDirectionPathUnKnown(int x, int y, int dir,
                                                        int now_dir,
                                                        int &nextDirection,
                                                        float &Value) {
  const bool isWall = existWall(x, y, dir);
  const bool step = isStep(x, y, dir);
  const float dist = isWall ? vector_max_step_val : getDistVector(x, y, dir);
  if (now_dir * dir == 8)
    return;
  if (!isWall && dist < Value) {
    nextDirection = dir;
    Value = dist;
  }
}

bool MazeSolverBaseLgc::arrival_goal_position(const int x, const int y) {
  for (const auto p : goal_list)
    if (p.x == x && p.y == y)
      return true;
  return false;
}

unsigned int MazeSolverBaseLgc::searchGoalPosition(const bool isSearch,
                                                   vector<point_t> &pt_list) {

  int next_dir = North;
  int now_dir = North;
  int x = 0;
  int y = 0;
  int position = 0;
  int idx;
  float Value;

  point_t pt;
  pt.x = 0;
  pt.y = 0;
  search_log.clear();
  unsigned int cnt = updateVectorMap(isSearch);
  while (true) {
    now_dir = next_dir;
    Value = VectorMax;
    next_dir = 255;
    pt.x = x;
    pt.y = y;
    search_log.push_back(pt);

    if (arrival_goal_position(x, y))
      break;

    const unsigned int position = getDistVector(x, y, now_dir);

    setNextRootDirectionPathUnKnown(x, y, North, now_dir, next_dir, Value);
    setNextRootDirectionPathUnKnown(x, y, East, now_dir, next_dir, Value);
    setNextRootDirectionPathUnKnown(x, y, West, now_dir, next_dir, Value);
    setNextRootDirectionPathUnKnown(x, y, South, now_dir, next_dir, Value);

    if (next_dir == North) {
      if (is_unknown(x, y, North)) {
        pt.x = x;
        pt.y = y + 1;
        pt_list.push_back(pt);
      }
    } else if (next_dir == East) {
      if (is_unknown(x, y, East)) {
        pt.x = x + 1;
        pt.y = y;
        pt_list.push_back(pt);
      }
    } else if (next_dir == West) {
      if (is_unknown(x, y, West)) {
        pt.x = x - 1;
        pt.y = y;
        pt_list.push_back(pt);
      }
    } else if (next_dir == South) {
      if (is_unknown(x, y, South)) {
        pt.x = x;
        pt.y = y - 1;
        pt_list.push_back(pt);
      }
    }

    if (next_dir == North)
      y++;
    else if (next_dir == East)
      x++;
    else if (next_dir == West)
      x--;
    else if (next_dir == South)
      y--;

    if (next_dir == 255)
      break;
  }
  // pt_list.erase(std::unique(pt_list.begin(), pt_list.end()), pt_list.end());

  return cnt;
}

unsigned int MazeSolverBaseLgc::searchGoalPosition(
    const bool isSearch,
    unordered_map<unsigned int, unsigned char> &subgoal_list) {
  int next_dir = North;
  int now_dir = North;
  int x = 0;
  int y = 0;
  int position = 0;
  int idx;
  float Value;

  point_t pt;
  pt.x = 0;
  pt.y = 0;
  search_log.clear();
  unsigned int cnt = updateVectorMap(isSearch, subgoal_list);
  while (true) {
    now_dir = next_dir;
    Value = VectorMax;
    next_dir = 255;
    pt.x = x;
    pt.y = y;
    search_log.push_back(pt);

    if (arrival_goal_position(x, y))
      break;

    const unsigned int position = getDistVector(x, y, now_dir);

    setNextRootDirectionPathUnKnown(x, y, North, now_dir, next_dir, Value);
    setNextRootDirectionPathUnKnown(x, y, East, now_dir, next_dir, Value);
    setNextRootDirectionPathUnKnown(x, y, West, now_dir, next_dir, Value);
    setNextRootDirectionPathUnKnown(x, y, South, now_dir, next_dir, Value);

    if (next_dir == North) {
      if (is_unknown(x, y, North))
        subgoal_list[x + (y + 1) * maze_size] = 1;
    } else if (next_dir == East) {
      if (is_unknown(x, y, East))
        subgoal_list[x + 1 + (y)*maze_size] = 1;
    } else if (next_dir == West) {
      if (is_unknown(x, y, West))
        subgoal_list[x - 1 + (y)*maze_size] = 1;
    } else if (next_dir == South) {
      if (is_unknown(x, y, South))
        subgoal_list[x + (y - 1) * maze_size] = 1;
    }

    if (next_dir == North)
      y++;
    else if (next_dir == East)
      x++;
    else if (next_dir == West)
      x--;
    else if (next_dir == South)
      y--;

    if (next_dir == 255)
      break;
  }
  // pt_list.erase(std::unique(pt_list.begin(), pt_list.end()), pt_list.end());

  return cnt;
}
void MazeSolverBaseLgc::searchGoalPosition2(const bool isSearch,
                                            vector<point_t> &pt_list) {

  int c = 0;
  for (int i = 0; i < maze_size; i++)
    for (int j = 0; j < maze_size; j++)
      dist[c++] = max_step_val;
  int head = 0;
  int tail = 0;
  for (const auto g : goal_list) {
    int idx = g.x + g.y * maze_size;
    dist[idx] = 0;
    q_list[tail].x = g.x;
    q_list[tail].y = g.y;
    tail++;
  }
  for (const auto g : goal_list3) {
    int idx = g.x + g.y * maze_size;
    dist[idx] = 0;
    q_list[tail].x = g.x;
    q_list[tail].y = g.y;
    tail++;
  }
  char D = 1;
  int pt1;
  int b;
  char X = 0, Y = 0;
  char i, j;
  point_t pt;
  while (head != tail) {
    Y = q_list[head].y;
    X = q_list[head].x;
    if (X == 0 && Y == 0)
      return;
    head++;
    pt1 = get_dist_val(X, Y) + 1;
    D = 1;
    while (D <= 8) {
      i = 0;
      j = 0;
      if (D == North)
        j = 1;
      else if (D == East)
        i = 1;
      else if (D == West)
        i = -1;
      else if (D == South)
        j = -1;

      b = !existWall(X, Y, D);

      if (b && get_dist_val(X + i, Y + j) == max_step_val) {
        if (X + i < 0 || X + i >= maze_size || Y + j < 0 ||
            Y + j >= maze_size) {
        } else {
          set_dist_val(X + i, Y + j, pt1);
          q_list[tail].x = X + i;
          q_list[tail].y = Y + j;
          tail++;
          if (is_unknown(X + i, Y + j, D)) {
            pt.x = X + i;
            pt.y = Y + j;
            pt_list.push_back(pt);
          }
        }
      }
      D *= 2;
    }
  }
}

MazeSolverBaseLgc::MazeSolverBaseLgc(/* args */) {}

MazeSolverBaseLgc::~MazeSolverBaseLgc() {}