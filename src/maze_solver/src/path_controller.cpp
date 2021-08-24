#include "path_controller.hpp"

PathController::PathController(/* args */) {}

PathController::~PathController() {}

void PathController::maze_callback(const my_msg::mazeConstPtr &mz) {
  int idx = 0;
  for (int i = 0; i < maze_size; i++) {
    for (int j = 0; j < maze_size; j++) {
      pc.lgc->set_map_val(j + i * maze_size, mz->wall[i + j * maze_size]);
    }
  }
  if (mz->cost_mode == 0) {
    pc.lgc->set_param1();
  } else if (mz->cost_mode == 1) {
    pc.lgc->set_param1();
  } else if (mz->cost_mode == 2) {
    pc.lgc->set_param2();
  } else if (mz->cost_mode == 3) {
    pc.lgc->set_param3();
  } else if (mz->cost_mode == 4) {
    pc.lgc->set_param4();
  } else if (mz->cost_mode == 5) {
    pc.lgc->set_param5();
  }
  pc.path_create(mz->motion != 0);
  my_msg::path path;
  for (const auto p : pc.path_s) {
    path.path_s.push_back(p);
  }
  for (const auto p : pc.path_t) {
    path.path_t.push_back(p);
  }
  pub_path.publish(path);
}

void PathController::set_meta_data() {
  XmlRpc::XmlRpcValue prm;

  _nh.getParam("/maze_data", prm);

  if (!(prm["wall"].valid() && prm["maze_size"].valid())) {
    ROS_WARN("rosparam /maze_data is not found");
    return;
  }
  maze_size = static_cast<int>(prm["maze_size"]);
  int max_step_val = static_cast<int>(prm["max_step_val"]);

  goal_list.clear();
  int goal_size = prm["goal"].size();
  for (int i = 0; i < goal_size; i++) {
    tmp_goal_p.x = static_cast<int>(prm["goal"][i][0]);
    tmp_goal_p.y = static_cast<int>(prm["goal"][i][1]);
    goal_list.push_back(tmp_goal_p);
  }

  // 迷路情報格納モジュール初期化
  pc.lgc = new MazeSolverBaseLgc();
  pc.lgc->init(maze_size, max_step_val);
  pc.lgc->set_goal_pos(goal_list);
}
void PathController::init() {
  set_meta_data();
  sub_maze = _nh.subscribe("/maze", 10, &PathController::maze_callback, this);
  pub_path = _nh.advertise<my_msg::path>("/path_info", 1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_creator");
  ros::NodeHandle nh;
  PathController pc;
  pc.init();

  ros::spin();
  return 0;
}