#include "path_controller.hpp"

PathController::PathController(/* args */) {}

PathController::~PathController() {}

void PathController::maze_callback(const my_msg::mazeConstPtr &mz) {
  pc.lgc = new MazeSolverBaseLgc();
  pc.lgc->init(maze_size, max_step_val);
  pc.lgc->set_goal_pos(goal_list);
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
    path.path_s.emplace_back(p);
  }
  for (const auto p : pc.path_t) {
    path.path_t.emplace_back(p);
  }
  pub_path.publish(path);

  // pc.convert_large_path(true);
  // pc.diagonalPath(false, true);

  my_msg::path path2;
  int size = pc.path_s.size();
  for (int i = 0; i < size; i++) {
    path2.path_s.emplace_back(pc.path_s[i]);
    path2.path_t.emplace_back(pc.path_t[i]);
    if (pc.path_t[i] == 255)
      break;
  }
  pub_path_dia.publish(path2);

  my_msg::base_path bp;
  my_msg::base_path_element ele;
  bp.paths.clear();
  bp.size = 0;
  for (int i = 0; i < size; i++) {
    ele.s = pc.path_s[i];
    ele.t = pc.path_t[i];
    bp.paths.emplace_back(ele);
    bp.size = i + 1;
    if (pc.path_t[i] == 255)
      break;
  }
  pub_base_path_dia.publish(bp);
}

void PathController::set_meta_data() {
  XmlRpc::XmlRpcValue prm;

  _nh.getParam("/maze_data", prm);

  if (!(prm["wall"].valid() && prm["maze_size"].valid())) {
    ROS_WARN("rosparam /maze_data is not found");
    return;
  }
  maze_size = static_cast<int>(prm["maze_size"]);
  max_step_val = static_cast<int>(prm["max_step_val"]);

  goal_list.clear();
  int goal_size = prm["goal"].size();
  for (int i = 0; i < goal_size; i++) {
    tmp_goal_p.x = static_cast<int>(prm["goal"][i][0]);
    tmp_goal_p.y = static_cast<int>(prm["goal"][i][1]);
    goal_list.emplace_back(tmp_goal_p);
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
  pub_path_dia = _nh.advertise<my_msg::path>("/path_info_dia", 1);
  pub_base_path_dia = _nh.advertise<my_msg::base_path>("/base_path_dia", 1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_creator");
  ros::NodeHandle nh;
  PathController pc;
  pc.init();

  ros::spin();
  return 0;
}