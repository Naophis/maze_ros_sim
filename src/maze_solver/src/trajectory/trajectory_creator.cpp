
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
TrajectoryCreator::TrajectoryCreator(/* args */) {}

TrajectoryCreator::~TrajectoryCreator() {}