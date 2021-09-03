
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

TurnDirection TrajectoryCreator::get_turn_dir(int turn_dir) {
  if (turn_dir == 255 || turn_dir == 0)
    return TurnDirection::End;
  return (turn_dir & 0x01) == 0x01 ? TurnDirection::Right : TurnDirection::Left;
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
    return 97;
  else if (type == TurnType::Orval)
    return 130;
  else if (type == TurnType::Large)
    return 190;
  else if (type == TurnType::Dia45)
    return 255;
  else if (type == TurnType::Dia135)
    return 130;
  else if (type == TurnType::Dia90)
    return 140;
  else if (type == TurnType::Kojima)
    return 180;

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
                                TurnDirection turn_dir) {
  if (type == TurnType::Normal) {
    if (ego.dir == Direction::North) {
      ego.y += 90;
      if (turn_dir == TurnDirection::Right)
        ego.x += 90;
      else
        ego.x -= 90;
    } else if (ego.dir == Direction::East) {
      ego.x += 90;
      if (turn_dir == TurnDirection::Right)
        ego.y -= 90;
      else
        ego.y += 90;
    } else if (ego.dir == Direction::West) {
      ego.x -= 90;
      if (turn_dir == TurnDirection::Right)
        ego.y += 90;
      else
        ego.y -= 90;
    } else if (ego.dir == Direction::South) {
      ego.y -= 90;
      if (turn_dir == TurnDirection::Right)
        ego.x -= 90;
      else
        ego.x += 90;
    }
  } else if (type == TurnType::Orval) {
    if (ego.dir == Direction::North) {
      if (turn_dir == TurnDirection::Right)
        ego.x += 180;
      else
        ego.x -= 180;
    } else if (ego.dir == Direction::East) {
      if (turn_dir == TurnDirection::Right)
        ego.y -= 180;
      else
        ego.y += 180;
    } else if (ego.dir == Direction::West) {
      if (turn_dir == TurnDirection::Right)
        ego.y += 180;
      else
        ego.y -= 180;
    } else if (ego.dir == Direction::South) {
      if (turn_dir == TurnDirection::Right)
        ego.x -= 180;
      else
        ego.x += 180;
    }
  } else if (type == TurnType::Large) {
    if (ego.dir == Direction::North) {
      ego.y += 180;
      if (turn_dir == TurnDirection::Right)
        ego.x += 180;
      else
        ego.x -= 180;
    } else if (ego.dir == Direction::East) {
      ego.x += 180;
      if (turn_dir == TurnDirection::Right)
        ego.y -= 180;
      else
        ego.y += 180;
    } else if (ego.dir == Direction::West) {
      ego.x -= 180;
      if (turn_dir == TurnDirection::Right)
        ego.y += 180;
      else
        ego.y -= 180;
    } else if (ego.dir == Direction::South) {
      ego.y -= 180;
      if (turn_dir == TurnDirection::Right)
        ego.x -= 180;
      else
        ego.x += 180;
    }
  } else if (type == TurnType::Dia45) {
    if (ego.dir == Direction::North) {
      ego.y += 180;
      if (turn_dir == TurnDirection::Right)
        ego.x += 90;
      else
        ego.x -= 90;
    } else if (ego.dir == Direction::East) {
      ego.x += 180;
      if (turn_dir == TurnDirection::Right)
        ego.y -= 90;
      else
        ego.y += 90;
    } else if (ego.dir == Direction::West) {
      ego.x -= 180;
      if (turn_dir == TurnDirection::Right)
        ego.y += 90;
      else
        ego.y -= 90;
    } else if (ego.dir == Direction::South) {
      ego.y -= 180;
      if (turn_dir == TurnDirection::Right)
        ego.x -= 90;
      else
        ego.x += 90;
    } else if (ego.dir == Direction::NorthEast) {
      if (turn_dir == TurnDirection::Right) {
        ego.x += 180;
        ego.y += 90;
      } else {
        ego.x += 90;
        ego.y += 180;
      }
    } else if (ego.dir == Direction::NorthWest) {
      if (turn_dir == TurnDirection::Right) {
        ego.x -= 90;
        ego.y += 180;
      } else {
        ego.x -= 180;
        ego.y += 80;
      }
    } else if (ego.dir == Direction::SouthEast) {
      if (turn_dir == TurnDirection::Right) {
        ego.x += 90;
        ego.y -= 180;
      } else {
        ego.x += 180;
        ego.y -= 90;
      }
    } else if (ego.dir == Direction::SouthWest) {
      if (turn_dir == TurnDirection::Right) {
        ego.x -= 180;
        ego.y -= 90;
      } else {
        ego.x -= 90;
        ego.y -= 180;
      }
    }
  } else if (type == TurnType::Dia135) {
    if (ego.dir == Direction::North) {
      ego.y += 90;
      if (turn_dir == TurnDirection::Right)
        ego.x += 180;
      else
        ego.x -= 180;
    } else if (ego.dir == Direction::East) {
      ego.x += 90;
      if (turn_dir == TurnDirection::Right)
        ego.y -= 180;
      else
        ego.y += 180;
    } else if (ego.dir == Direction::West) {
      ego.x -= 90;
      if (turn_dir == TurnDirection::Right)
        ego.y += 180;
      else
        ego.y -= 180;
    } else if (ego.dir == Direction::South) {
      ego.y -= 90;
      if (turn_dir == TurnDirection::Right)
        ego.x -= 180;
      else
        ego.x += 180;
    } else if (ego.dir == Direction::NorthEast) {
      if (turn_dir == TurnDirection::Right) {
        ego.x += 180;
        ego.y -= 90;
      } else {
        ego.x -= 90;
        ego.y += 180;
      }
    } else if (ego.dir == Direction::NorthWest) {
      if (turn_dir == TurnDirection::Right) {
        ego.x += 90;
        ego.y += 180;
      } else {
        ego.x -= 180;
        ego.y -= 90;
      }
    } else if (ego.dir == Direction::SouthEast) {
      if (turn_dir == TurnDirection::Right) {
        ego.x -= 90;
        ego.y -= 180;
      } else {
        ego.x += 180;
        ego.y += 90;
      }
    } else if (ego.dir == Direction::SouthWest) {
      if (turn_dir == TurnDirection::Right) {
        ego.x -= 180;
        ego.y += 90;
      } else {
        ego.x += 90;
        ego.y -= 180;
      }
    }
  } else if (type == TurnType::Dia90) {
    if (ego.dir == Direction::NorthEast) {
      if (turn_dir == TurnDirection::Right) {
        ego.x += 180;
      } else {
        ego.y += 180;
      }
    } else if (ego.dir == Direction::NorthWest) {
      if (turn_dir == TurnDirection::Right) {
        ego.y += 180;
      } else {
        ego.x -= 180;
      }
    } else if (ego.dir == Direction::SouthEast) {
      if (turn_dir == TurnDirection::Right) {
        ego.y -= 180;
      } else {
        ego.x += 180;
      }
    } else if (ego.dir == Direction::SouthWest) {
      if (turn_dir == TurnDirection::Right) {
        ego.x -= 180;
      } else {
        ego.y -= 180;
      }
    }
  }
}
void TrajectoryCreator::exec2(path_struct &base_path,
                              vector<trajectory_point_t> &trajectory) {
  trajectory.clear();
  
  trajectory_point_t trj_ele = {0};
  float v_max = 2000;
  const double dt = 0.001;
  ego_odom_t ego;
  ego.x = ego.y = ego.ang = 0;
  ego.dir = Direction::North;
  bool dia = false;

  TurnDirection turn_dir = TurnDirection::None;
  TurnType turn_type = TurnType::None;

  for (const auto bp : base_path.paths) {
    float dist = 0.5 * bp.s * 180;
    float dist2 = 0.5 * bp.s - 1;
    int turn = bp.t;
    trj_ele = {0};

    turn_dir = get_turn_dir(bp.t);
    turn_type = get_turn_type(bp.t);

    if (dist > 0 && dist2 > 0) {
      dist = !dia ? (dist2 * 180) : (dist2 * 180 * ROOT2);

      float tmp_dist = 0;
      trj_ele.ang = ego.ang;
      trj_ele.x = ego.x;
      trj_ele.y = ego.y;

      trj_ele.ang = get_run_dir(ego.dir, trj_ele.ang);
      while (tmp_dist < dist) {
        trj_ele.x = trj_ele.x + v_max * sin(trj_ele.ang) * dt;
        trj_ele.y = trj_ele.y + v_max * cos(trj_ele.ang) * dt;
        tmp_dist += v_max * dt;
        trajectory.emplace_back(trj_ele);
      }
      ego.x = trj_ele.x;
      ego.y = trj_ele.y;
      ego.ang = trj_ele.ang;
    }

    if (!((turn_type == TurnType::None) || (turn_type == TurnType::Finish))) {
      float tgt_ang = get_turn_tgt_ang(turn_type);
      float radius = get_turn_rad(turn_type);
      float alpha = 2 * v_max * v_max / (radius * radius * tgt_ang / 2);

      if (turn_dir == TurnDirection::Left)
        alpha *= -1;

      float w = 0;
      trj_ele.ang = ego.ang;
      trj_ele.x = ego.x;
      trj_ele.y = ego.y;

      float tmp_ang = 0;
      while (abs(tmp_ang) < tgt_ang) {
        if (abs(tmp_ang) >= tgt_ang / 2)
          w += -alpha * dt;
        else
          w += alpha * dt;

        trj_ele.ang += w * dt;
        tmp_ang += abs(w) * dt;
        trj_ele.x = trj_ele.x + v_max * sin(trj_ele.ang) * dt;
        trj_ele.y = trj_ele.y + v_max * cos(trj_ele.ang) * dt;
        trajectory.emplace_back(trj_ele);
      }
      // ego.x = trj_ele.x;
      // ego.y = trj_ele.y;
      fix_pos(ego, turn_type, turn_dir);
      ego.dir = get_next_dir(ego.dir, turn_type, turn_dir);
      if (ego.dir == Direction::NorthEast || ego.dir == Direction::NorthWest ||
          ego.dir == Direction::SouthEast || ego.dir == Direction::SouthWest) {
        dia = true;
      } else {
        dia = false;
      }
      ego.ang = trj_ele.ang;
      trj_ele.ang = ego.ang;
      trj_ele.x = ego.x;
      trj_ele.y = ego.y;
      trajectory.emplace_back(trj_ele);
    }
  }
}
TrajectoryCreator::TrajectoryCreator(/* args */) {}

TrajectoryCreator::~TrajectoryCreator() {}