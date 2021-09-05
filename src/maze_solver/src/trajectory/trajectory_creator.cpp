
#include "trajectory_creator.hpp"

TurnType TrajectoryCreator::get_turn_type(int turn_num) {
  if (turn_num == 1 || turn_num == 2)
    return TurnType::Normal;
  if (turn_num == 3 || turn_num == 4)
    return TurnType::Orval;
  if (turn_num == 5 || turn_num == 6)
    return TurnType::Large;
  if (turn_num == 7 || turn_num == 8)
    return TurnType::Dia45;
  if (turn_num == 9 || turn_num == 10)
    return TurnType::Dia135;
  if (turn_num == 11 || turn_num == 12)
    return TurnType::Dia90;
  if (turn_num == 13 || turn_num == 14)
    return TurnType::Kojima;
  if (turn_num == 255)
    return TurnType::Finish;
  return TurnType::Finish;
}

TurnDirection TrajectoryCreator::get_turn_dir(int turn_num) {
  if (turn_num == 255 || turn_num == 0)
    return TurnDirection::End;
  return (turn_num & 0x01) == 0x01 ? TurnDirection::Right : TurnDirection::Left;
}
float TrajectoryCreator::get_run_dir(Direction dir, float ang) {
  if (dir == Direction::North)
    return 0;
  else if (dir == Direction::East)
    return 3.14 / 2;
  else if (dir == Direction::West)
    return -3.14 / 2;
  else if (dir == Direction::South)
    return 3.14;
  else if (dir == Direction::NorthEast)
    return 3.14 / 4;
  else if (dir == Direction::NorthWest)
    return -3.14 / 4;
  else if (dir == Direction::SouthEast)
    return 3.14 * 3 / 4;
  else if (dir == Direction::SouthWest)
    return -3.14 * 3 / 4;
  return ang;
}

float TrajectoryCreator::get_turn_tgt_ang(TurnType type) {
  if (type == TurnType::Normal)
    return 3.14 / 2;
  else if (type == TurnType::Orval)
    return 3.14;
  else if (type == TurnType::Large)
    return 3.14 / 2;
  else if (type == TurnType::Dia45)
    return 3.14 / 4;
  else if (type == TurnType::Dia135)
    return 3.14 * 3 / 4;
  else if (type == TurnType::Dia90)
    return 3.14 / 2;
  else if (type == TurnType::Kojima)
    return 3.14 / 2;
  return 0;
}

float TrajectoryCreator::get_turn_rad(TurnType type) {
  if (type == TurnType::Normal)
    return sla_data.normal.radius;
  else if (type == TurnType::Orval)
    return sla_data.orval.radius;
  else if (type == TurnType::Large)
    return sla_data.large.radius;
  else if (type == TurnType::Dia45)
    return sla_data.dia45.radius;
  else if (type == TurnType::Dia135)
    return sla_data.dia135.radius;
  else if (type == TurnType::Dia90)
    return sla_data.dia90.radius;
  // else if (type == TurnType::Kojima)
  //   return 90;
  return 0;
}

float TrajectoryCreator::get_front_dist(TurnType type, bool dia) {
  if (type == TurnType::Normal)
    return sla_data.normal.front;
  else if (type == TurnType::Orval)
    return sla_data.orval.front;
  else if (type == TurnType::Large)
    return sla_data.large.front;
  else if (type == TurnType::Dia45)
    return !dia ? sla_data.dia45.front : sla_data.dia45_2.front;
  else if (type == TurnType::Dia135)
    return !dia ? sla_data.dia135.front : sla_data.dia135_2.front;
  else if (type == TurnType::Dia90)
    return sla_data.dia90.front;
  // else if (type == TurnType::Kojima)
  //   return 90;
  return 0;
}

float TrajectoryCreator::get_back_dist(TurnType type, bool dia) {
  if (type == TurnType::Normal)
    return sla_data.normal.back;
  else if (type == TurnType::Orval)
    return sla_data.orval.back;
  else if (type == TurnType::Large)
    return sla_data.large.back;
  else if (type == TurnType::Dia45)
    return !dia ? sla_data.dia45.back : sla_data.dia45_2.back;
  else if (type == TurnType::Dia135)
    return !dia ? sla_data.dia135.back : sla_data.dia135_2.back;
  else if (type == TurnType::Dia90)
    return sla_data.dia90.back;
  // else if (type == TurnType::Kojima)
  //   return 90;
  return 0;
}
float TrajectoryCreator::get_slalom_etn(TurnType type, bool dia) {
  if (type == TurnType::Normal)
    return sla_data.normal.n;
  else if (type == TurnType::Orval)
    return sla_data.orval.n;
  else if (type == TurnType::Large)
    return sla_data.large.n;
  else if (type == TurnType::Dia45)
    return !dia ? sla_data.dia45.n : sla_data.dia45_2.n;
  else if (type == TurnType::Dia135)
    return !dia ? sla_data.dia135.n : sla_data.dia135_2.n;
  else if (type == TurnType::Dia90)
    return sla_data.dia90.n;
  // else if (type == TurnType::Kojima)
  //   return 90;
  return 0;
}
float TrajectoryCreator::get_slalom_time(TurnType type, bool dia) {
  if (type == TurnType::Normal)
    return sla_data.normal.time;
  else if (type == TurnType::Orval)
    return sla_data.orval.time;
  else if (type == TurnType::Large)
    return sla_data.large.time;
  else if (type == TurnType::Dia45)
    return !dia ? sla_data.dia45.time : sla_data.dia45_2.time;
  else if (type == TurnType::Dia135)
    return !dia ? sla_data.dia135.time : sla_data.dia135_2.time;
  else if (type == TurnType::Dia90)
    return sla_data.dia90.time;
  // else if (type == TurnType::Kojima)
  //   return 90;
  return 0;
}

Direction TrajectoryCreator::get_next_dir(Direction dir, TurnType type,
                                          TurnDirection turn_dir) {

  if (type == TurnType::Normal || type == TurnType::Large) {
    if (dir == Direction::North)
      return turn_dir == TurnDirection::Right ? Direction::East
                                              : Direction::West;
    else if (dir == Direction::East)
      return turn_dir == TurnDirection::Right ? Direction::South
                                              : Direction::North;
    else if (dir == Direction::West)
      return turn_dir == TurnDirection::Right ? Direction::North
                                              : Direction::South;
    else if (dir == Direction::South)
      return turn_dir == TurnDirection::Right ? Direction::West
                                              : Direction::East;
  } else if (type == TurnType::Orval) {
    if (dir == Direction::North)
      return Direction::South;
    else if (dir == Direction::East)
      return Direction::West;
    else if (dir == Direction::West)
      return Direction::East;
    else if (dir == Direction::South)
      return Direction::North;
  } else if (type == TurnType::Dia45) {
    if (dir == Direction::North)
      return turn_dir == TurnDirection::Right ? Direction::NorthEast
                                              : Direction::NorthWest;
    else if (dir == Direction::East)
      return turn_dir == TurnDirection::Right ? Direction::SouthEast
                                              : Direction::NorthEast;
    else if (dir == Direction::West)
      return turn_dir == TurnDirection::Right ? Direction::NorthWest
                                              : Direction::SouthWest;
    else if (dir == Direction::South)
      return turn_dir == TurnDirection::Right ? Direction::SouthWest
                                              : Direction::SouthEast;
    else if (dir == Direction::NorthEast)
      return turn_dir == TurnDirection::Right ? Direction::East
                                              : Direction::North;
    else if (dir == Direction::NorthWest)
      return turn_dir == TurnDirection::Right ? Direction::North
                                              : Direction::West;
    else if (dir == Direction::SouthEast)
      return turn_dir == TurnDirection::Right ? Direction::South
                                              : Direction::East;
    else if (dir == Direction::SouthWest)
      return turn_dir == TurnDirection::Right ? Direction::West
                                              : Direction::South;
  } else if (type == TurnType::Dia135) {
    if (dir == Direction::North)
      return turn_dir == TurnDirection::Right ? Direction::SouthEast
                                              : Direction::SouthWest;
    else if (dir == Direction::East)
      return turn_dir == TurnDirection::Right ? Direction::SouthWest
                                              : Direction::NorthWest;
    else if (dir == Direction::West)
      return turn_dir == TurnDirection::Right ? Direction::NorthEast
                                              : Direction::SouthEast;
    else if (dir == Direction::South)
      return turn_dir == TurnDirection::Right ? Direction::NorthWest
                                              : Direction::NorthEast;
    else if (dir == Direction::NorthEast)
      return turn_dir == TurnDirection::Right ? Direction::South
                                              : Direction::West;
    else if (dir == Direction::NorthWest)
      return turn_dir == TurnDirection::Right ? Direction::East
                                              : Direction::South;
    else if (dir == Direction::SouthEast)
      return turn_dir == TurnDirection::Right ? Direction::West
                                              : Direction::North;
    else if (dir == Direction::SouthWest)
      return turn_dir == TurnDirection::Right ? Direction::North
                                              : Direction::East;
  } else if (type == TurnType::Dia90) {
    if (dir == Direction::NorthEast)
      return turn_dir == TurnDirection::Right ? Direction::SouthEast
                                              : Direction::NorthWest;
    else if (dir == Direction::NorthWest)
      return turn_dir == TurnDirection::Right ? Direction::NorthEast
                                              : Direction::SouthWest;
    else if (dir == Direction::SouthEast)
      return turn_dir == TurnDirection::Right ? Direction::SouthWest
                                              : Direction::NorthEast;
    else if (dir == Direction::SouthWest)
      return turn_dir == TurnDirection::Right ? Direction::NorthWest
                                              : Direction::SouthEast;
  }
  return dir;
}
void TrajectoryCreator::fix_pos(ego_odom_t &ego, TurnType type,
                                TurnDirection turn_dir,
                                trajectory_point_t &trj_ele) {
  if (type == TurnType::Normal) {
    if (ego.dir == Direction::North) {
      ego.y += half_cell_size;
      if (turn_dir == TurnDirection::Right)
        ego.x += half_cell_size;
      else
        ego.x -= half_cell_size;
    } else if (ego.dir == Direction::East) {
      ego.x += half_cell_size;
      if (turn_dir == TurnDirection::Right)
        ego.y -= half_cell_size;
      else
        ego.y += half_cell_size;
    } else if (ego.dir == Direction::West) {
      ego.x -= half_cell_size;
      if (turn_dir == TurnDirection::Right)
        ego.y += half_cell_size;
      else
        ego.y -= half_cell_size;
    } else if (ego.dir == Direction::South) {
      ego.y -= half_cell_size;
      if (turn_dir == TurnDirection::Right)
        ego.x -= half_cell_size;
      else
        ego.x += half_cell_size;
    }
  } else if (type == TurnType::Orval) {
    if (ego.dir == Direction::North) {
      if (turn_dir == TurnDirection::Right)
        ego.x += cell_size;
      else
        ego.x -= cell_size;
    } else if (ego.dir == Direction::East) {
      if (turn_dir == TurnDirection::Right)
        ego.y -= cell_size;
      else
        ego.y += cell_size;
    } else if (ego.dir == Direction::West) {
      if (turn_dir == TurnDirection::Right)
        ego.y += cell_size;
      else
        ego.y -= cell_size;
    } else if (ego.dir == Direction::South) {
      if (turn_dir == TurnDirection::Right)
        ego.x -= cell_size;
      else
        ego.x += cell_size;
    }
  } else if (type == TurnType::Large) {
    if (ego.dir == Direction::North) {
      ego.y += cell_size;
      if (turn_dir == TurnDirection::Right)
        ego.x += cell_size;
      else
        ego.x -= cell_size;
    } else if (ego.dir == Direction::East) {
      ego.x += cell_size;
      if (turn_dir == TurnDirection::Right)
        ego.y -= cell_size;
      else
        ego.y += cell_size;
    } else if (ego.dir == Direction::West) {
      ego.x -= cell_size;
      if (turn_dir == TurnDirection::Right)
        ego.y += cell_size;
      else
        ego.y -= cell_size;
    } else if (ego.dir == Direction::South) {
      ego.y -= cell_size;
      if (turn_dir == TurnDirection::Right)
        ego.x -= cell_size;
      else
        ego.x += cell_size;
    }
  } else if (type == TurnType::Dia45) {
    if (ego.dir == Direction::North) {
      ego.y += cell_size;
      if (turn_dir == TurnDirection::Right)
        ego.x += half_cell_size;
      else
        ego.x -= half_cell_size;
    } else if (ego.dir == Direction::East) {
      ego.x += cell_size;
      if (turn_dir == TurnDirection::Right)
        ego.y -= half_cell_size;
      else
        ego.y += half_cell_size;
    } else if (ego.dir == Direction::West) {
      ego.x -= cell_size;
      if (turn_dir == TurnDirection::Right)
        ego.y += half_cell_size;
      else
        ego.y -= half_cell_size;
    } else if (ego.dir == Direction::South) {
      ego.y -= cell_size;
      if (turn_dir == TurnDirection::Right)
        ego.x -= half_cell_size;
      else
        ego.x += half_cell_size;
    } else if (ego.dir == Direction::NorthEast) {
      if (turn_dir == TurnDirection::Right) {
        ego.x += cell_size;
        ego.y += half_cell_size;
      } else {
        ego.x += half_cell_size;
        ego.y += cell_size;
      }
    } else if (ego.dir == Direction::NorthWest) {
      if (turn_dir == TurnDirection::Right) {
        ego.x -= half_cell_size;
        ego.y += cell_size;
      } else {
        ego.x -= cell_size;
        ego.y += half_cell_size;
      }
    } else if (ego.dir == Direction::SouthEast) {
      if (turn_dir == TurnDirection::Right) {
        ego.x += half_cell_size;
        ego.y -= cell_size;
      } else {
        ego.x += cell_size;
        ego.y -= half_cell_size;
      }
    } else if (ego.dir == Direction::SouthWest) {
      if (turn_dir == TurnDirection::Right) {
        ego.x -= cell_size;
        ego.y -= half_cell_size;
      } else {
        ego.x -= half_cell_size;
        ego.y -= cell_size;
      }
    }
  } else if (type == TurnType::Dia135) {
    if (ego.dir == Direction::North) {
      ego.y += half_cell_size;
      if (turn_dir == TurnDirection::Right)
        ego.x += cell_size;
      else
        ego.x -= cell_size;
    } else if (ego.dir == Direction::East) {
      ego.x += half_cell_size;
      if (turn_dir == TurnDirection::Right)
        ego.y -= cell_size;
      else
        ego.y += cell_size;
    } else if (ego.dir == Direction::West) {
      ego.x -= half_cell_size;
      if (turn_dir == TurnDirection::Right)
        ego.y += cell_size;
      else
        ego.y -= cell_size;
    } else if (ego.dir == Direction::South) {
      ego.y -= half_cell_size;
      if (turn_dir == TurnDirection::Right)
        ego.x -= cell_size;
      else
        ego.x += cell_size;
    } else if (ego.dir == Direction::NorthEast) {
      if (turn_dir == TurnDirection::Right) {
        ego.x += cell_size;
        ego.y -= half_cell_size;
      } else {
        ego.x -= half_cell_size;
        ego.y += cell_size;
      }
    } else if (ego.dir == Direction::NorthWest) {
      if (turn_dir == TurnDirection::Right) {
        ego.x += half_cell_size;
        ego.y += cell_size;
      } else {
        ego.x -= cell_size;
        ego.y -= half_cell_size;
      }
    } else if (ego.dir == Direction::SouthEast) {
      if (turn_dir == TurnDirection::Right) {
        ego.x -= half_cell_size;
        ego.y -= cell_size;
      } else {
        ego.x += cell_size;
        ego.y += half_cell_size;
      }
    } else if (ego.dir == Direction::SouthWest) {
      if (turn_dir == TurnDirection::Right) {
        ego.x -= cell_size;
        ego.y += half_cell_size;
      } else {
        ego.x += half_cell_size;
        ego.y -= cell_size;
      }
    }
  } else if (type == TurnType::Dia90) {
    if (ego.dir == Direction::NorthEast) {
      if (turn_dir == TurnDirection::Right)
        ego.x += cell_size;
      else
        ego.y += cell_size;
    } else if (ego.dir == Direction::NorthWest) {
      if (turn_dir == TurnDirection::Right)
        ego.y += cell_size;
      else
        ego.x -= cell_size;
    } else if (ego.dir == Direction::SouthEast) {
      if (turn_dir == TurnDirection::Right)
        ego.y -= cell_size;
      else
        ego.x += cell_size;
    } else if (ego.dir == Direction::SouthWest) {
      if (turn_dir == TurnDirection::Right)
        ego.x -= cell_size;
      else
        ego.y -= cell_size;
    }
  }
  ego.ang = trj_ele.ang;
  trj_ele.ang = ego.ang;
  trj_ele.x = ego.x;
  trj_ele.y = ego.y;
}
void TrajectoryCreator::exec2(path_struct &base_path,
                              vector<trajectory_point_t> &trajectory) {
  trajectory.clear();

  trajectory_point_t trj_ele = {0};
  float v_max = 1500;
  float v_max2 = 5000;
  ego_odom_t ego;
  ego.x = ego.y = ego.ang = 0;
  ego.dir = Direction::North;
  bool dia = false;

  TurnDirection turn_dir = TurnDirection::None;
  TurnType turn_type = TurnType::None;
  run_param_t param = {0};
  param.v_max = 5000;
  param.turn_v = 1500;
  for (const auto bp : base_path.paths) {
    float dist = 0.5 * bp.s * 180;
    float dist2 = 0.5 * bp.s - 1;
    int turn = bp.t;
    trj_ele = {0};

    turn_dir = get_turn_dir(bp.t);
    turn_type = get_turn_type(bp.t);

    if (dist > 0 && dist2 > 0) {
      dist = !dia ? (dist2 * 180) : (dist2 * 180 * ROOT2);
      run_straight(trajectory, dist, trj_ele, ego, param, 0);
    }

    if (!((turn_type == TurnType::None) || (turn_type == TurnType::Finish))) {

      trj_ele.ang = ego.ang;
      trj_ele.x = ego.x;
      trj_ele.y = ego.y;

      // 前距離
      dist = get_front_dist(turn_type, dia);
      if (dist > 0)
        run_straight(trajectory, dist, trj_ele, ego, param, 1);

      // 旋回
      slalom(trajectory, bp.t, trj_ele, ego, param, dia);

      // 後距離
      dist = get_back_dist(turn_type, dia);
      if (dist > 0)
        run_straight(trajectory, dist, trj_ele, ego, param, 3);

      if (true) {
        fix_pos(ego, turn_type, turn_dir, trj_ele);
      } else {
        ego.x = trj_ele.x;
        ego.y = trj_ele.y;
        ego.ang = trj_ele.ang;
      }
      ego.dir = get_next_dir(ego.dir, turn_type, turn_dir);

      dia =
          (ego.dir == Direction::NorthEast || ego.dir == Direction::NorthWest ||
           ego.dir == Direction::SouthEast || ego.dir == Direction::SouthWest);

      trajectory.emplace_back(trj_ele);
    }
  }
}

float TrajectoryCreator::Et2(float t, float s, float N) {
  const float z = 1.0f;
  t = t / s;
  float P = pow((t - z), N - z);
  float Q = P * (t - z);
  return -N * P / ((Q - z) * (Q - z)) * pow(exp(1), z + z / (Q - z)) / s;
}

void TrajectoryCreator::run_straight(vector<trajectory_point_t> &trajectory,
                                     float dist, trajectory_point_t &trj_ele,
                                     ego_odom_t &ego, run_param_t &param,
                                     char type) {
  float tmp_dist = 0;
  float v_max = param.v_max;

  if (type == 0) {
    trj_ele.ang = ego.ang;
    trj_ele.x = ego.x;
    trj_ele.y = ego.y;
    trj_ele.ang = get_run_dir(ego.dir, trj_ele.ang);
  }
  trj_ele.type = type;

  while (tmp_dist < dist) {
    trj_ele.x = trj_ele.x + v_max * sin(trj_ele.ang) * dt;
    trj_ele.y = trj_ele.y + v_max * cos(trj_ele.ang) * dt;
    tmp_dist += v_max * dt;
    trajectory.emplace_back(trj_ele);
  }

  if (type == 0) {
    ego.x = trj_ele.x;
    ego.y = trj_ele.y;
    ego.ang = trj_ele.ang;
  }
}
void TrajectoryCreator::slalom(vector<trajectory_point_t> &trajectory,
                               int turn_num, trajectory_point_t &trj_ele,
                               ego_odom_t &ego, run_param_t &param, bool dia) {
  float tmp_ang = 0;
  int sinCount = 1;

  float v_max = param.turn_v;
  TurnType turn_type = get_turn_type(turn_num);
  TurnDirection turn_dir = get_turn_dir(turn_num);
  float tgt_ang = get_turn_tgt_ang(turn_type);
  float radius = get_turn_rad(turn_type);
  float slaTerm = get_slalom_time(turn_type, dia);
  float etN = get_slalom_etn(turn_type, dia);
  float alphaTemp = v_max / radius;
  float w = 0;
  float alpha = 0;

  trj_ele.type = 2;
  if (turn_dir == TurnDirection::Left)
    alphaTemp *= -1;

  while (abs(tmp_ang) < tgt_ang) {
    if (dt * sinCount / slaTerm < 2.0) {
      alpha = alphaTemp * Et2(dt * sinCount, slaTerm, etN);
    } else {
      alpha = 0;
      w = 0;
      break;
    }
    w += alpha * dt;
    tmp_ang += abs(w) * dt;
    trj_ele.ang += w * dt;
    trj_ele.x = trj_ele.x + v_max * sin(trj_ele.ang) * dt;
    trj_ele.y = trj_ele.y + v_max * cos(trj_ele.ang) * dt;
    trajectory.emplace_back(trj_ele);
    sinCount++;
  }
}
TrajectoryCreator::TrajectoryCreator(/* args */) {}

TrajectoryCreator::~TrajectoryCreator() {}