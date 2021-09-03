
#include "trajectory_creator.hpp"

void TrajectoryCreator::exec(path_struct &base_path,
                             vector<trajectory_point_t> &trajectory) {
  trajectory.clear();
  trajectory_point_t trj_ele = {0};
  float v_max = 1000;
  const double dt = 0.001;

  for (const auto bp : base_path.paths) {
    float dist = 0.5 * bp.s * 180;
    int turn = bp.t;
    trj_ele = {0};
    if (turn == static_cast<int>(PathMotion::End)) {
      float tmp_dist = 0;
      while (tmp_dist < dist) {
        trj_ele.x = 0;
        trj_ele.y = trj_ele.y + v_max * dt;
        trj_ele.ang = 0;
        tmp_dist += v_max * dt;
        trajectory.emplace_back(trj_ele);
      }
      break;
    } else if (turn == static_cast<int>(PathMotion::Right)) {
      float tgt_ang = 3.14 / 2;
      float radius = 90;
      float alpha = 2 * v_max * v_max / (radius * radius * tgt_ang / 2);
      float w = 0;
      while (abs(trj_ele.ang) < tgt_ang) {
        if (abs(trj_ele.ang) >= tgt_ang / 2) {
          w += -alpha * dt;
        } else {
          w += alpha * dt;
        }
        trj_ele.ang += w * dt;
        trj_ele.x = trj_ele.x + v_max * sin(trj_ele.ang) * dt;
        trj_ele.y = trj_ele.y + v_max * cos(trj_ele.ang) * dt;
        trajectory.emplace_back(trj_ele);
      }
    } else if (turn == static_cast<int>(PathMotion::Left)) {
      float tgt_ang = 3.14 / 2;
      float radius = 90;
      float alpha = -2 * v_max * v_max / (radius * radius * tgt_ang / 2);
      float w = 0;
      while (abs(trj_ele.ang) < tgt_ang) {
        if (abs(trj_ele.ang) >= tgt_ang / 2) {
          w += -alpha * dt;
        } else {
          w += alpha * dt;
        }
        trj_ele.ang += w * dt;
        trj_ele.x = trj_ele.x + v_max * sin(trj_ele.ang) * dt;
        trj_ele.y = trj_ele.y + v_max * cos(trj_ele.ang) * dt;
        trajectory.emplace_back(trj_ele);
      }
    }
    return;
  }
}
void TrajectoryCreator::exec2(path_struct &base_path,
                              vector<trajectory_point_t> &trajectory) {
  trajectory.clear();
  trajectory_point_t trj_ele = {0};
  float v_max = 100;
  const double dt = 0.001;
  ego_odom_t ego;
  ego.x = ego.y = ego.ang = 0;
  ego.dir = Direction::North;

  for (const auto bp : base_path.paths) {
    float dist = 0.5 * bp.s * 180;
    float dist2 = 0.5 * bp.s - 1;
    int turn = bp.t;
    trj_ele = {0};

    if (dist > 0 && dist2 > 0) {
      dist = dist2 * 180;
      float tmp_dist = 0;
      trj_ele.ang = ego.ang;
      trj_ele.x = ego.x;
      trj_ele.y = ego.y;
      while (tmp_dist < dist) {
        if (ego.dir == Direction::North) {
          trj_ele.x = trj_ele.x;
          trj_ele.y = trj_ele.y + v_max * dt;
          trj_ele.ang = 0;
        } else if (ego.dir == Direction::East) {
          trj_ele.x = trj_ele.x + v_max * dt;
          trj_ele.y = trj_ele.y;
          trj_ele.ang = 3.14 / 2;
        } else if (ego.dir == Direction::West) {
          trj_ele.x = trj_ele.x - v_max * dt;
          trj_ele.y = trj_ele.y;
          trj_ele.ang = -3.14 / 2;
        } else if (ego.dir == Direction::South) {
          trj_ele.x = trj_ele.x;
          trj_ele.y = trj_ele.y - v_max * dt;
          trj_ele.ang = 3.14;
        }
        tmp_dist += v_max * dt;
        trajectory.emplace_back(trj_ele);
      }

      if (ego.dir == Direction::North) {
        ego.x = ego.x;
        ego.y = ego.y + dist;
      } else if (ego.dir == Direction::East) {
        ego.x = ego.x + dist;
        ego.y = ego.y;
      } else if (ego.dir == Direction::West) {
        ego.x = ego.x - dist;
        ego.y = ego.y;
      } else if (ego.dir == Direction::South) {
        ego.x = ego.x;
        ego.y = ego.y - dist;
      }
      ego.ang = trj_ele.ang;

      trj_ele.ang = ego.ang;
      trj_ele.x = ego.x;
      trj_ele.y = ego.y;
      trajectory.emplace_back(trj_ele);
    } else {
      // dist = 0;
      // if (ego.dir == Direction::North) {
      //   ego.x = ego.x;
      //   ego.y = ego.y + dist;
      // } else if (ego.dir == Direction::East) {
      //   ego.x = ego.x + dist;
      //   ego.y = ego.y;
      // } else if (ego.dir == Direction::West) {
      //   ego.x = ego.x - dist;
      //   ego.y = ego.y;
      // } else if (ego.dir == Direction::South) {
      //   ego.x = ego.x;
      //   ego.y = ego.y - dist;
      // }
      // ego.ang = trj_ele.ang;

      // trj_ele.ang = ego.ang;
      // trj_ele.x = ego.x;
      // trj_ele.y = ego.y;
      // trajectory.emplace_back(trj_ele);
    }

    if (turn == static_cast<int>(PathMotion::Right)) {
      float tgt_ang = 3.14 / 2;
      float radius = 90;
      float alpha = 2 * v_max * v_max / (radius * radius * tgt_ang / 2);
      float w = 0;
      trj_ele.ang = ego.ang;
      trj_ele.x = ego.x;
      trj_ele.y = ego.y;
      float tmp_ang = 0;
      while (abs(tmp_ang) < tgt_ang) {
        if (abs(tmp_ang) >= tgt_ang / 2) {
          w += -alpha * dt;
        } else {
          w += alpha * dt;
        }
        trj_ele.ang += w * dt;
        tmp_ang += w * dt;
        trj_ele.x = trj_ele.x + v_max * sin(trj_ele.ang) * dt;
        trj_ele.y = trj_ele.y + v_max * cos(trj_ele.ang) * dt;
        trajectory.emplace_back(trj_ele);
      }
      // ego.x = trj_ele.x;
      // ego.y = trj_ele.y;
      // ego.ang = trj_ele.ang;
      if (ego.dir == Direction::North) {
        ego.dir = Direction::East;
        ego.ang = 3.14 / 2;
        ego.x = ego.x + 90;
        ego.y = ego.y + 90;
      } else if (ego.dir == Direction::East) {
        ego.dir = Direction::South;
        ego.ang = 3.14;
        ego.x = ego.x + 90;
        ego.y = ego.y - 90;
      } else if (ego.dir == Direction::West) {
        ego.dir = Direction::North;
        ego.ang = 0;
        ego.x = ego.x - 90;
        ego.y = ego.y + 90;
      } else if (ego.dir == Direction::South) {
        ego.dir = Direction::West;
        ego.ang = -3.14 / 2;
        ego.x = ego.x - 90;
        ego.y = ego.y - 90;
      }

      trj_ele.ang = ego.ang;
      trj_ele.x = ego.x;
      trj_ele.y = ego.y;
      trajectory.emplace_back(trj_ele);

    } else if (turn == static_cast<int>(PathMotion::Left)) {
      float tgt_ang = 3.14 / 2;
      float radius = 90;
      float alpha = -2 * v_max * v_max / (radius * radius * tgt_ang / 2);
      float w = 0;
      trj_ele.ang = ego.ang;
      trj_ele.x = ego.x;
      trj_ele.y = ego.y;
      float tmp_ang = 0;
      while (abs(tmp_ang) < tgt_ang) {
        if (abs(tmp_ang) >= tgt_ang / 2) {
          w += -alpha * dt;
        } else {
          w += alpha * dt;
        }
        trj_ele.ang += w * dt;
        tmp_ang += w * dt;
        trj_ele.x = trj_ele.x + v_max * sin(trj_ele.ang) * dt;
        trj_ele.y = trj_ele.y + v_max * cos(trj_ele.ang) * dt;
        trajectory.emplace_back(trj_ele);
      }
      if (ego.dir == Direction::North) {
        ego.dir = Direction::West;
        ego.ang = -3.14 / 2;
        ego.x = ego.x - 90;
        ego.y = ego.y + 90;
      } else if (ego.dir == Direction::East) {
        ego.dir = Direction::North;
        ego.ang = 0;
        ego.x = ego.x + 90;
        ego.y = ego.y + 90;
      } else if (ego.dir == Direction::West) {
        ego.dir = Direction::South;
        ego.ang = 3.14;
        ego.x = ego.x - 90;
        ego.y = ego.y - 90;
      } else if (ego.dir == Direction::South) {
        ego.dir = Direction::East;
        ego.ang = 3.14 / 2;
        ego.x = ego.x + 90;
        ego.y = ego.y - 90;
      }
      trj_ele.ang = ego.ang;
      trj_ele.x = ego.x;
      trj_ele.y = ego.y;
      trajectory.emplace_back(trj_ele);
    }
    // return;
  }
}
TrajectoryCreator::TrajectoryCreator(/* args */) {}

TrajectoryCreator::~TrajectoryCreator() {}