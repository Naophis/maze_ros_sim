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
  path_s.emplace_back(val); //
}
void PathCreator::append_path_t(int val) {
  path_t.emplace_back(val); //
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

bool PathCreator::path_create(bool is_search) {
  bool use;
  return path_create(is_search, 0, 0, Direction::Null, use);
}

bool PathCreator::path_create(bool is_search, int tgt_x, int tgt_y,
                              Direction tgt_dir, bool &use) {
  Direction next_dir = Direction::North;
  Direction now_dir = next_dir;
  unsigned int idx = 0;
  Direction dirLog[3];

  int x = 0;
  int y = 1;

  path_reflash();
  clearCheckMap();
  lgc->updateVectorMap(is_search);

  path_s.emplace_back(3);
  float dist_val = MAX;
  float old_dist_val = MAX;
  unordered_map<int, int> stepped;
  int cnt = 0;
  while (true) {
    cnt++;
    if (cnt > 1023) {
      return false;
    }
    float old_dist_val = dist_val;
    now_dir = next_dir;
    dirLog[2] = dirLog[1];
    dirLog[1] = dirLog[0];
    dirLog[0] = now_dir;
    next_dir = Direction::Undefined;

    if (stepped.count(x + lgc->maze_size * y) != 0) {
      return false;
    }
    stepped[x + lgc->maze_size * y] = 1;

    if (lgc->arrival_goal_position(x, y)) {
      add_path_s(idx, 3);
      path_t.emplace_back(255);
      // path_t.emplace_back(0);
      path_size = idx;
      return true;
    }
    float position = lgc->VectorMax;
    if (now_dir == Direction::North) {
      position = lgc->getDistVector(x, y, Direction::South);
    } else if (now_dir == Direction::East) {
      position = lgc->getDistVector(x, y, Direction::West);
    } else if (now_dir == Direction::West) {
      position = lgc->getDistVector(x, y, Direction::East);
    } else if (now_dir == Direction::South) {
      position = lgc->getDistVector(x, y, Direction::North);
    }

    setNextRootDirectionPath(x, y, now_dir, Direction::North, dist_val,
                             next_dir);
    setNextRootDirectionPath(x, y, now_dir, Direction::East, dist_val,
                             next_dir);
    setNextRootDirectionPath(x, y, now_dir, Direction::West, dist_val,
                             next_dir);
    setNextRootDirectionPath(x, y, now_dir, Direction::South, dist_val,
                             next_dir);
    if (dist_val == old_dist_val) {
      break;
    }
    if (dirLog[0] == dirLog[1] || dirLog[0] != dirLog[2])
      priorityStraight2(x, y, now_dir, dirLog[0], dist_val, next_dir);
    else
      priorityStraight2(x, y, now_dir, dirLog[1], dist_val, next_dir);

    if (tgt_dir != Direction::Null) {
      if (x == tgt_x && y == tgt_y) {
        next_dir = tgt_dir;

        dist_val = lgc->getDistV(x, y, next_dir);
        use = true;
      }
      if (other_route_map.count(x + lgc->maze_size * y) != 0) {
        //ルート決定してたら強制設定
        if (other_route_map[x + lgc->maze_size * y].selected) {
          next_dir = other_route_map[x + lgc->maze_size * y].select_dir;
          dist_val = lgc->getDistV(x, y, next_dir);
        }
      }
    }

    Motion nextMotion = get_next_motion(now_dir, next_dir);
    checkOtherRoot(x, y, now_dir, position);
    if (nextMotion == Motion::Straight) {
      add_path_s(idx, 2);
    } else if (nextMotion == Motion::TurnRight) {
      path_t.emplace_back(static_cast<int>(TurnDirection::Right));
      path_s.emplace_back(2);
      idx++;
    } else if (nextMotion == Motion::TurnLeft) {
      path_t.emplace_back(static_cast<int>(TurnDirection::Left));
      path_s.emplace_back(2);
      idx++;
    } else {
      path_t.emplace_back(0);
      break;
    }
    next_dir = get_next_pos(x, y, now_dir, next_dir);
  }
  path_size = idx;
  return false;
}

void PathCreator::convert_large_path(bool b1) {
  int i = 0;
  int finish = 0;
  for (i = 0; i < path_size; i++) {
    if (path_s[i + 1] == 2) {
      if ((path_s[i] > 2) && path_s[i + 2] > 2) {
        if (path_t[i] == path_t[i + 1]) {
          if (path_t[i] == 1) {
            path_t[i] = 3;
            path_t[i + 1] = 254;
          } else if (path_t[i] == 2) {
            path_t[i] = 4;
            path_t[i + 1] = 254;
          }
          path_s[i] -= 1;
          path_s[i + 2] -= 1;
          i++;
        }
      }
    }
    if (path_t[i] == 0) {
      break;
    }
  }
  for (i = 0; i < path_size; i++) {
    if ((path_s[i] > 2) && (path_s[i + 1] > 2)) {
      if (path_t[i] == 1) {
        path_t[i] = 5;
        path_s[i + 1] -= 1;
        path_s[i] -= 1;
      } else if (path_t[i] == 2) {
        path_t[i] = 6;
        path_s[i + 1] -= 1;
        path_s[i] -= 1;
      }
    }
    if (path_t[i] == 0) {
      break;
    }
  }
  if (b1) {
    if (path_size >= 2 && path_s[0] == 2 && path_t[0] == 1 && path_s[1] > 2) {
      path_s[0] -= 1;
      path_s[1] -= 1;
      path_t[0] = 5;
    }
    i = 0;
    while (path_t[i] != 0) {
      i++;
    }
    finish = i;
    if (finish >= 1) {
      if (path_s[finish] == 2 && path_s[finish - 1] > 2 &&
          (path_t[finish - 1] == 1 || path_t[finish - 1] == 2)) {
        path_s[finish - 1] -= 1;
        path_s[finish] -= 1;
        if (path_t[finish - 1] == 1) {
          path_t[finish - 1] = 5;
        } else if (path_t[finish - 1] == 2) {
          path_t[finish - 1] = 6;
        }
      }
    }
    if (finish > 2) {
      if (path_s[finish] == 2 && path_s[finish - 1] == 2 &&
          path_s[finish - 2] > 2 && path_t[finish - 1] == 1 &&
          path_t[finish - 2] == 1) {
        path_s[finish - 2] -= 1;
        path_s[finish] -= 1;
        path_t[finish - 2] = 3;
        path_t[finish - 1] = 254;
      }
      if (path_s[finish] == 2 && path_s[finish - 1] == 2 &&
          path_s[finish - 2] > 2 && path_t[finish - 1] == 2 &&
          path_t[finish - 2] == 2) {
        path_s[finish - 2] -= 1;
        path_s[finish] -= 1;
        path_t[finish - 2] = 4;
        path_t[finish - 1] = 254;
      }
    }
    if (finish == 2) {
      if (path_s[finish] == 2 && path_s[finish - 1] == 2 &&
          path_s[finish - 2] >= 2 && path_t[finish - 1] == 1 &&
          path_t[finish - 2] == 1) {
        path_s[finish - 2] -= 1;
        path_s[finish] -= 1;
        path_t[finish - 2] = 3;
        path_t[finish - 1] = 254;
      }
      if (path_s[finish] == 2 && path_s[finish - 1] == 2 &&
          path_s[finish - 2] >= 2 && path_t[finish - 1] == 2 &&
          path_t[finish - 2] == 2) {
        path_s[finish - 2] -= 1;
        path_s[finish] -= 1;
        path_t[finish - 2] = 4;
        path_t[finish - 1] = 254;
      }
    }
  }
}

void PathCreator::diagonalPath(bool isFull, bool a1) {
  int i = 0;
  int j = 0;
  int dir = 0;
  int check = 0;
  bool diaMode = false;
  int m = 0;
  bool check2 = false;
  bool check3 = false;

  bool _virtual = !a1;
  bool flag = false;
  while (path_t[i] != 0) {
    check = 0;
    if (_virtual) {
      a1 = false;
      if (path_s[i] > 2) {
        flag = true;
      }
      if (flag /* && i != 0*/) {
        for (m = i + 1;; m++) {
          if (path_s[m] > 2) {
            a1 = true;
            break;
          }
          if (path_t[m] == 0) {
            a1 = false;
            break;
          }
        }
      }
    }

    //		if (/*i > 0 &&*/m != 0 && path_s[m] > 2) {
    //			a1 = true;
    //		}
    if (path_t[i] == R && (a1 ? true : path_s[i] > 2)) {
      dir = R;
      for (j = i + 1; path_t[j] != dir; j++) {
        if (path_t[j] == R && path_s[j] == 2) {
          dir = R;
        } else if (path_t[j] == L && path_s[j] == 2) {
          dir = L;
        } else {
          break;
        }
        check++;
      }
    } else if (path_t[i] == L && (a1 ? true : path_s[i] > 2)) {
      dir = L;
      for (j = i + 1; path_t[j] != dir; j++) {
        if (path_t[j] == R && path_s[j] == 2) {
          dir = R;
        } else if (path_t[j] == L && path_s[j] == 2) {
          dir = L;
        } else {
          break;
        }
        check++;
      }
    }
    if (check != 0) {
      j -= 1;
      if ((i != 0 && path_s[i] == 2 && path_t[i] == path_t[i - 1] &&
           path_s[i - 1] > 2)) {
        //      ||    (i == 1 && path_s[i] == 2 && path_t[i] == path_t[i - 1] &&
        //      path_s[i - 1] > 0)) {
        if (path_t[i] == R) {
          path_t[i - 1] = 9;
          check3 = true;
        } else if (path_t[i] == L) {
          path_t[i - 1] = 10;
          check3 = true;
        }
        if (j != 0 && path_s[j + 2] > 2 && path_t[j] == path_t[j + 1]) {
          if (path_t[j] == R) {
            path_t[j + 1] = 9;
            check2 = true;
          } else if (path_t[j] == L) {
            path_t[j + 1] = 10;
            check2 = true;
          }
          path_s[j + 1] = check + 1;
        } else {
          if (path_t[j] == R) {
            path_t[j] = 7;
          } else if (path_t[j] == L) {
            path_t[j] = 8;
          }
          path_s[j] = check + 1;
        }
      } else {
        int memory = 0;
        if (path_t[i] == R) {
          path_t[i] = 7;
          memory = R;
        } else if (path_t[i] == L) {
          path_t[i] = 8;
          memory = L;
        }
        if (j != 0 && path_s[j + 2] > 2 && path_t[j] == path_t[j + 1]) {
          if (path_t[j] == R) {
            path_t[j + 1] = 9;
            check2 = true;
          } else if (path_t[j] == L) {
            path_t[j + 1] = 10;
            check2 = true;
          }
          path_s[j + 1] = check + 1;
        } else {
          if (!a1) {
            if (path_s[j + 1] > 2) {
              if (path_t[j] == R) {
                path_t[j] = 7;
              } else if (path_t[j] == L) {
                path_t[j] = 8;
              }
              path_s[j] = check + 1;
            } else {
              path_t[i] = memory;
              check3 = false;
              check2 = false;
              i = j;
              i++;
              continue;
            }
          } else {
            if (path_t[j] == R) {
              path_t[j] = 7;
            } else if (path_t[j] == L) {
              path_t[j] = 8;
            }
            path_s[j] = check + 1;
          }
        }
      }
      if (check3) {
        for (int k = i; k < j; k++) {
          path_t[k] = 254;
        }
      } else {
        for (int k = i + 1; k < j; k++) {
          path_t[k] = 254;
        }
      }
      if (check2) {
        path_t[j] = 254;
      }
      check3 = false;
      check2 = false;
      i = j;
    }
    i++;
  }
  i = 0;

  while (path_t[i] != 0) {
    if (path_t[i] == 7 && path_t[i + 1] == 7 && path_s[i + 1] == 2) {
      path_t[i] = 11;
      path_t[i + 1] = 254;
    }
    if (path_t[i] == 8 && path_t[i + 1] == 8 && path_s[i + 1] == 2) {
      path_t[i] = 12;
      path_t[i + 1] = 254;
    }
    i++;
  }
  i = 0;
  diaMode = false;
  while (path_t[i] != 0) {
    if (!diaMode) {
      if (path_t[i] == 7 || path_t[i] == 8 || path_t[i] == 9 ||
          path_t[i] == 10) {
        path_s[i] -= 1;
        diaMode = true;
      }
    } else if (diaMode) {
      if (path_t[i] == 7 || path_t[i] == 8 || path_t[i] == 9 ||
          path_t[i] == 10) {
        path_s[i + 1] -= 1;
        diaMode = false;
      }
    }
    i++;
  }
  pathOffset();
  // TODO APEC対応でコメントアウト中
  //	if (isFull) {
  //		i = 0;
  //		for (i = 0; i < 255; i++) {
  //			if (i > 0 && path_t[i] == 255 && path_s[i] == 1) {
  //				if (path_t[i - 1] == 7 || path_t[i - 1] == 8) {
  //					path_t[i - 1] = 255;
  //					path_s[i - 1] += 1;
  //				}
  //			}
  //			if (path_t[i] == 255) {
  //				break;
  //			}
  //		}
  //	}
}

void PathCreator::pathOffset() {
  int i = 0;
  for (i = 0; i < path_size; i++) {
    if (path_t[i] == 0) {
      path_t[i] = 0xff;
      break;
    }
  }
  i = 0;
  while (i < path_size && path_t[i] != 0xff) {
    if (i > 0) {
      if (path_t[i] == 254) {
        path_s[i] = 0;
        path_t[i] = 0;
      }
    }
    i++;
  }
  i = 0;
  while (i < path_size && path_t[i] != 0xff) {
    while (path_t[i] == 0) {
      for (int j = i; path_t[j] != 0xff; j++) {
        path_s[j] = path_s[j + 1];
        path_t[j] = path_t[j + 1];
      }
    }
    i++;
  }
}

bool PathCreator::path_create_with_change(bool is_search, int tgt_x, int tgt_y,
                                          Direction tgt_dir,
                                          path_create_status_t &pc_state) {
  //初期化
  pc_result.time = 10000;
  pc_result.use = false;

  pc_result.state =
      path_create(is_search, tgt_x, tgt_y, tgt_dir, pc_result.use);
  if (!pc_result.state) {
    return false;
  }
  convert_large_path(true);
  diagonalPath(false, true);
  float len_t = 0;
  float len_s = 0;

  //  for (int i = 0; i < path_t.size(); i++) {
  //    len_s += path_s[i];
  //    if (path_t[i] == 7 || path_t[i] == 8) {
  //      if ((path_s[i + 1] * 0.5 - 1) > 0) {
  //        len_t += 1;
  //      } else {
  //        len_t += 1;
  //      }
  //    }
  //    if (path_t[i] == 9 || path_t[i] == 10) {
  //      if ((path_s[i + 1] * 0.5 - 1) > 0) {
  //        len_t += 1.5;
  //      } else {
  //        len_t += 1.5;
  //      }
  //    } else if (path_t[i] == 11 || path_t[i] == 12) {
  //      len_t += 1;
  //    } else {
  //      len_t += 1;
  //    }
  //  }
  //  for (int i = 0; i < path_t.size(); i++) {
  //    len_t++;
  //    len_s += path_s[i];
  //    if (path_t[i] == 255) {
  //      break;
  //    }
  //  }
  // pc_result.time = len_s + len_t;
  pc_result.time = calc_goal_time();
  //    pc_result.time =  len_t;

  return true;
}

float PathCreator::drawChangePathRoot(bool is_search) {
  bool next_end = false;
  candidate_route_info_t tmp_cand_route;
  float time = 10000;

  for (int i = 0; i < 100; i++) { //何かでリミットを設ける
    const auto before = other_route_map.size();
    unordered_map<int, candidate_route_info_t>::iterator itr =
        other_route_map.begin();
    for (itr; itr != other_route_map.end(); itr++) {
      auto tmp_cand = (itr->second);
      if (tmp_cand.selected) {
        //決定済みなら、固定したルートだけ入れる
        tmp_cand.candidate_dir_set.clear();
        tmp_cand.candidate_dir_set.insert(tmp_cand.select_dir);
      } else {
        route_q.push((itr->second));
      }
    }
    while (!route_q.empty()) {
      const auto cand = route_q.top();
      route_q.pop();
      int x = cand.x;
      int y = cand.y;
      //初期化
      while (!route_list.empty()) {
        route_list.pop();
      }
      // 該当の位置で分岐してPathを形成。それぞれ評価。
      for (const auto dir : cand.candidate_dir_set) {
        bool goal = path_create_with_change(true, x, y, dir, pc_result);
        if (!pc_result.state || !goal) {
          // たどり着かない等、詰んでいたら元のセットから除外。
          other_route_map[x + lgc->maze_size * y].candidate_dir_set.erase(dir);
        } else {
          route.time = pc_result.time;
          if (time > route.time) {
            time = route.time;
            path_s2.clear();
            path_t2.clear();
            for (int i = 0; i < path_t.size(); i++) {
              path_s2.push_back(path_s[i]);
              path_t2.push_back(path_t[i]);
            }
          }
          route.dir = dir;
          route.use = pc_result.use;
          route_list.push(route);
        }
      }
      // priority queueで最良ルートがトップに来るので、それだけ使う。
      if (route_list.size() > 0) {
        const auto top = route_list.top();
        if (top.use) {
          //矯正指定して意味があったら固定する。意味がないなら何もしない。
          other_route_map[x + lgc->maze_size * y].selected = true;
          other_route_map[x + lgc->maze_size * y].select_dir = top.dir;
        }
      }
    }

    const auto after = other_route_map.size();
    // このアルゴリズムで辿れるルートの膨れ上がりが止まったら終了。一応for文でlimitは設ける。
    if (next_end) {
      break;
    }
    if (before == after) {
      //      break;
      next_end = true;
    }
  }
  return 0;
}
// 他のルートがあったらmapに保存する
void PathCreator::checkOtherRoot(int x, int y, Direction now_dir, float now) {
  //１度見つけてるやつはスルー
  if (other_route_map.count(x + y * lgc->maze_size) > 0) {
    return;
  }

  const int now_d = static_cast<int>(now_dir);
  candidate_route_info_t cand;
  //候補になる向きを探す。
  for (const auto d : direction_list) {
    const auto dist = lgc->getDistVector(x, y, d);
    const auto d_int = static_cast<int>(d);
    if (now_d * d_int != 8 && !lgc->existWall(x, y, d) && dist < now) {
      other_route_map[x + y * lgc->maze_size].candidate_dir_set.insert(d);
    }
  }
  //候補が１つなら削除
  if (other_route_map[x + y * lgc->maze_size].candidate_dir_set.size() <= 1) {
    other_route_map.erase(x + y * lgc->maze_size);
  } else {
    // 2つ以上あるなら正式登録
    other_route_map[x + y * lgc->maze_size].from_dist = now;
    other_route_map[x + y * lgc->maze_size].x = x;
    other_route_map[x + y * lgc->maze_size].y = y;
  }
}

float PathCreator::calc_goal_time() {
  bool fast_mode = false;
  bool start_turn = false;
  bool dia = false;
  float cell_size = 90;
  float v_now = 0;
  float time = 0;
  Direction ego_dir = Direction::North;
  for (int i = 0; i < path_t.size(); i++) {
    float dist = 0.5 * path_s[i] - 1;
    auto turn_dir = tc.get_turn_dir(path_t[i]);
    auto turn_type = tc.get_turn_type(path_t[i], dia);
    start_turn = false;
    if (dist > 0) {
      fast_mode = true;
    }
    if ((dist > 0) || i == 0) {
      auto st = !dia ? StraightType::FastRun : StraightType::FastRunDia;

      float v_max = 4000;
      float v_end = 1000;
      float accl = 12000;
      float decel = -12000;

      float dist2 = !dia ? (dist * cell_size) : (dist * cell_size * ROOT2);
      if (i == 0) {
        if (dist == 0) { // 初手ターンの場合は距離合成して加速区間を増やす
          dist = 10;
          start_turn = true;
        }
        dist2 += 18; // 初期加速距離を加算
        auto tmp_v2 = 2 * accl * dist2;
        if (v_end * v_end > tmp_v2) {
          accl = (v_end * v_end) / (2 * dist2) + 1000;
          decel = -accl;
        }
      }
      if (turn_type == TurnType::Finish) {
        dist -= 45;
        v_end = 500;
      }

      time += go_straight_dummy(v_now, v_max, v_end, accl, decel, dist2);
      v_now = v_end;
      if (turn_type == TurnType::Finish) {
        break;
      }
    }

    if (!((turn_type == TurnType::None) || (turn_type == TurnType::Finish))) {
      auto st = !dia ? StraightType::FastRun : StraightType::FastRunDia;
      bool exist_next_idx = (i + 1) < path_t.size(); //絶対true
      float dist3 = 0;
      float dist4 = 0;
      if (exist_next_idx) {
        dist3 = 0.5 * path_s[i + 1] * cell_size;
        dist4 = 0.5 * path_s[i + 1] - 1;
      }
      // スラロームの後距離の目標速度を指定

      float v_max = 4000;
      float v_end = 1000;
      float accl = 12000;
      float decel = -12000;
      float slalom_time = 0.2;

      time += slalom_dummy(turn_type);
      dia =
          (ego_dir == Direction::NorthEast || ego_dir == Direction::NorthWest ||
           ego_dir == Direction::SouthEast || ego_dir == Direction::SouthWest);
    }
  }
  return time;
}

char PathCreator::asc(float d, float d2) {
  if (d < d2) {
    return 2;
  }
  return 1;
}

float PathCreator::go_straight_dummy(float v1, float vmax, float v2, float ac,
                                     float diac, float dist) {

  double dt = 0.001;
  double acc = ac;
  double distance = 0;
  double time = 0;
  double V_now = v1;
  int sequence = 1;
  double d2;
  while (distance < dist) {
    time += dt;
    d2 = std::abs((V_now + v2) * (V_now - v2) / (2.0 * diac));
    switch (sequence) {
    case 3:
      acc = 0;
      break;
    case 1:
      sequence = asc(dist - distance, d2);
      if (V_now >= vmax) {
        acc = 0;
        V_now = vmax;
      } else {
        acc = ac;
      }
      if (sequence != 3) {
        break;
      }
    case 2:
      if (V_now <= v2) {
        acc = 0;
        V_now = v2;
      } else {
        acc = -diac;
      }
      break;
    }
    V_now += acc * dt;
    distance += V_now * dt;
  }

  return time;
}
float PathCreator::slalom_dummy(TurnType turn_type) {
  if (turn_type == TurnType::Normal) {
    return 0.112813 * 2;
  } else if (turn_type == TurnType::Orval) {
    return 0.061749999999994934 * 2;
  } else if (turn_type == TurnType::Large) {
    return 0.061749999999994934 * 2;
  } else if (turn_type == TurnType::Dia45) {
    return 0.033968749999997994 * 2;
  } else if (turn_type == TurnType::Dia135) {
    return 0.07409374999999872 * 2;
  } else if (turn_type == TurnType::Dia45_2) {
    return 0.042953124999997004 * 2;
  } else if (turn_type == TurnType::Dia135_2) {
    return 0.061749999999994934 * 2;
  } else if (turn_type == TurnType::Dia90) {
    return 0.037046874999997655 * 2;
  }
  return 0.112813 * 2;
}