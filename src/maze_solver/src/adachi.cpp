#include "adachi.hpp"

Adachi::Adachi(/* args */) {}

Adachi::~Adachi() {}

void Adachi::set_logic(MazeSolverBaseLgc *_logic) { lgc = _logic; }

void Adachi::set_ego(ego_t *_ego) { ego = _ego; }

void Adachi::setNextDirection(int x2, int y2, int dir, int &next_dir,
                              int &val) {

  bool isWall = lgc->existWall(ego->x, ego->y, dir);
  bool step = lgc->isStep(x2, y2, dir);
  bool step2 = (lgc->isStep(x2, y2, North) && lgc->isStep(x2, y2, East) &&
                lgc->isStep(x2, y2, West) && lgc->isStep(x2, y2, South));
  unsigned int dist = lgc->get_dist_val(x2, y2);

  bool bool1 = goaled && (pt_list.size() == 1);

  if (!goaled || is_go_home() || (bool1)) {
    if (!isWall && !step && dist < val) {
      next_dir = dir;
      val = dist;
    } else if (!isWall && step && dist < val) {
      next_dir = dir;
      val = dist;
    }
  } else {
    //  if (!isWall && !step2) {
    //    next_dir = dir;
    //    val = dist; //未探索有線固定
    //  } else
    if (!isWall && !step && dist < val) {
      next_dir = dir;
      val = dist;
    } else if (!isWall && step && dist < val) {
      next_dir = dir;
      val = dist;
    }
  }
}

bool Adachi::is_goal(int x, int y) {
  for (const auto p : lgc->goal_list)
    if (p.x == x && p.y == 0)
      return true;
  return false;
}

void Adachi::deadEnd() {
  bool head = true;
  while (head) {
    head = false;
    for (char i = -1; i <= 1; i++) {
      for (char j = -1; j <= 1; j++) {
        const unsigned char x = ego->x + i;
        const unsigned char y = ego->y + j;
        const unsigned char temp = lgc->get_map_val(x, y) & 0x0f;
        if (x == 0 && y == 0)
          continue;
        if (is_goal(x, y))
          continue;
        if (ego->x == x && ego->y == y)
          continue;

        if (temp == 0x07) {
          lgc->updateWall(x, y, South);
          head = true;
        }
        if (temp == 0x0b) {
          lgc->updateWall(x, y, West);
          head = true;
        }
        if (temp == 0x0d) {
          lgc->updateWall(x, y, East);
          head = true;
        }
        if (temp == 0x0e) {
          lgc->updateWall(x, y, North);
          head = true;
        }
      }
    }
  }
}

int Adachi::detect_next_direction() {
  int dist_val = lgc->get_max_step_val();
  int next_dir = 255;
  const bool enable_back = lgc->get_dist_val(ego->x, ego->y) > limit2;
  const bool stop = (!is_go_home() && ego->x == 0 && ego->y == 1);
  if (ego->dir == North) {
    setNextDirection(ego->x, ego->y + 1, North, next_dir, dist_val);
    setNextDirection(ego->x + 1, ego->y, East, next_dir, dist_val);
    setNextDirection(ego->x - 1, ego->y, West, next_dir, dist_val);
    if (enable_back || is_go_home() || stop)
      setNextDirection(ego->x, ego->y - 1, South, next_dir, dist_val);
  } else if (ego->dir == East) {
    setNextDirection(ego->x + 1, ego->y, East, next_dir, dist_val);
    setNextDirection(ego->x, ego->y - 1, South, next_dir, dist_val);
    setNextDirection(ego->x, ego->y + 1, North, next_dir, dist_val);
    if (enable_back || is_go_home() || stop)
      setNextDirection(ego->x - 1, ego->y, West, next_dir, dist_val);
  } else if (ego->dir == West) {
    setNextDirection(ego->x - 1, ego->y, West, next_dir, dist_val);
    setNextDirection(ego->x, ego->y + 1, North, next_dir, dist_val);
    setNextDirection(ego->x, ego->y - 1, South, next_dir, dist_val);
    if (enable_back || is_go_home() || stop)
      setNextDirection(ego->x + 1, ego->y, East, next_dir, dist_val);
  } else if (ego->dir == South) {
    setNextDirection(ego->x, ego->y - 1, South, next_dir, dist_val);
    setNextDirection(ego->x - 1, ego->y, West, next_dir, dist_val);
    setNextDirection(ego->x + 1, ego->y, East, next_dir, dist_val);
    if (enable_back || is_go_home() || stop)
      setNextDirection(ego->x, ego->y + 1, North, next_dir, dist_val);
  }
  return next_dir;
}

void Adachi::get_next_pos(int next_direction) {
  if (next_direction == 255)
    if (ego->dir == North)
      next_direction = South;
    else if (ego->dir == East)
      next_direction = West;
    else if (ego->dir == West)
      next_direction = East;
    else if (ego->dir == South)
      next_direction = North;

  if (next_direction == North)
    ego->y++;
  else if (next_direction == East)
    ego->x++;
  else if (next_direction == West)
    ego->x--;
  else if (next_direction == South)
    ego->y--;

  ego->dir = next_direction;
  return;
}

int Adachi::get_next_motion(int next_direction) {
  if (ego->dir == next_direction)
    return Straight;

  if (ego->dir == North) {
    if (next_direction == East)
      return Right;
    else if (next_direction == West)
      return Left;
  } else if (ego->dir == East) {
    if (next_direction == South)
      return Right;
    else if (next_direction == North)
      return Left;
  } else if (ego->dir == West) {
    if (next_direction == North)
      return Right;
    else if (next_direction == South)
      return Left;
  } else if (ego->dir == South) {
    if (next_direction == West)
      return Right;
    else if (next_direction == East)
      return Left;
  }
  return Back;
}

bool Adachi::is_go_home() {
  if (pt_list.size() == 1 && pt_list[0].x == 0 && pt_list[0].y == 0)
    return true;
  return false;
}

int Adachi::exec() {
  int goal_size = lgc->goal_list.size();
  int goaled_cnt = 0;
  int status = 0;
  // check
  goaled_cnt = 0;
  for (const auto g : lgc->goal_list)
    if (lgc->is_stepped(g.x, g.y))
      goaled_cnt++;

  int calc_cnt = 0;
  goaled = (goal_size == goaled_cnt);

  if (!goaled) {
    pt_list.clear();
  }
  lgc->update_dist_map(0, goaled); // search
  deadEnd();
  if (goaled) {
    if (!goal_startpos_lock) {
      if (lgc->is_stepped(next_goal_pt.x, next_goal_pt.y) ||
          lgc->candidate_end(next_goal_pt.x, next_goal_pt.y) ||
          lgc->get_dist_val(next_goal_pt.x, next_goal_pt.y) > limit) {

        if (pt_list.size() == 0) {
          lgc->set_param1();
          calc_cnt += lgc->searchGoalPosition(true, pt_list);
          cost_mode = 1;
        }
        if (pt_list.size() == 0) {
          lgc->set_param2();
          calc_cnt += lgc->searchGoalPosition(true, pt_list);
          cost_mode = 2;
        }
        // if (pt_list.size() == 0) {
        //   lgc->set_param5();
        //   calc_cnt += lgc->searchGoalPosition(true, pt_list);
        //   cost_mode = 5;
        // }
        // if (pt_list.size() == 0) {
        //   lgc->set_param3();
        //   calc_cnt += lgc->searchGoalPosition(true, pt_list);
        //   cost_mode = 3;
        // }

        // if (pt_list.size() == 0) {
        //   lgc->set_param4();
        //   calc_cnt += lgc->searchGoalPosition(true, pt_list);
        //   cost_mode = 4;
        // }

        if (pt_list.size() == 0) {
          pt_list.clear();
          point_t tmp_p;
          tmp_p.x = tmp_p.y = 0;
          pt_list.push_back(tmp_p);
        }

        lgc->set_goal_pos2(pt_list);
        lgc->update_dist_map(0, goaled); // 再更新したらもう1回歩数マップ生成
      }
    }
    if (pt_list.size() == 1 && pt_list[0].x == 0 && pt_list[0].y == 0)
      goal_startpos_lock = true; //ゴール固定
  }

  if (goal_startpos_lock)
    if (ego->x == 0 && ego->y == 0) {
      calc_cnt += lgc->searchGoalPosition(false, pt_list);
      lgc->update_dist_map(1, false); // search
      return 0;
    }

  int next_dir = detect_next_direction();
  int next_motion = get_next_motion(next_dir);
  get_next_pos(next_dir);
  lgc->remove_goal_pos3();

  ROS_INFO("calc_cnt = %d", calc_cnt);
  return next_motion;
}