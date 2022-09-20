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
  pc.other_route_map.clear();
  pc.path_create(mz->motion != 0);
  pc.convert_large_path(true);
  pc.diagonalPath(false, true);
  pc.path_s2.clear();
  pc.path_t2.clear();
  for (int i = 0; i < pc.path_t.size(); i++) {
    pc.path_s2.push_back(pc.path_s[i]);
    pc.path_t2.push_back(pc.path_t[i]);
  }
  pc.drawChangePathRoot(mz->motion != 0);

  // my_msg::path path;
  // for (const auto p : pc.path_s) {
  //   path.path_s.emplace_back(p);
  // }
  // for (const auto p : pc.path_t) {
  //   path.path_t.emplace_back(p);
  // }
  // pub_path.publish(path);

  //  pc.convert_large_path(true);
  //  pc.diagonalPath(false, true);

  my_msg::path path2;
  int size = pc.path_s2.size();
    path2.path_s.clear();
    path2.path_t.clear();
  for (int i = 0; i < size; i++) {
    path2.path_s.push_back(pc.path_s2[i]);
    path2.path_t.push_back(pc.path_t2[i]);
    if(pc.path_t2[i]==2 || pc.path_t2[i]==1){
        ROS_WARN("check");
    }
    if (pc.path_t2[i] == 255)
      break;
  }
  pub_path_dia.publish(path2);

  my_msg::base_path bp;
  my_msg::base_path_element ele;
  bp.paths.clear();
  bp.size = 0;
  for (int i = 0; i < size; i++) {
    ele.s = pc.path_s2[i];
    ele.t = pc.path_t2[i];
    bp.paths.emplace_back(ele);
    bp.size = i + 1;
    if (pc.path_t2[i] == 255)
      break;
  }
  pub_base_path_dia.publish(bp);
  my_msg::dist_map cell;
  my_msg::dist_map_q q;
  q.size = 0;
  const auto start = pc.lgc->get_diadist_n_val(0, 0);
  ROS_WARN("other_route_map_size = %d", pc.other_route_map.size());
  for (const auto c : pc.lgc->memo_q) {
    cell.x = c.x;
    cell.y = c.y;
    cell.dir = static_cast<int>(c.dir);
    cell.dist = c.dist2;
    q.cell_list.emplace_back(cell);
    q.size++;
  }
  pub_dist_map_q.publish(q);

  my_msg::candidate_route_map cand_map;
  for (const auto cand : pc.other_route_map) {
    my_msg::candidate_cell_set cand_set;
    for (const auto dir : cand.second.candidate_dir_set) {
      cand_set.dir.emplace_back(static_cast<int>(dir));
    }
    cand_set.x = cand.second.x;
    cand_set.y = cand.second.y;
    cand_map.map.emplace_back(cand_set);
  }
  pub_candidate_map.publish(cand_map);
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
  pub_dist_map_q = _nh.advertise<my_msg::dist_map_q>("/dist_map_q", 1);
  pub_candidate_map =
      _nh.advertise<my_msg::candidate_route_map>("/candidate_route_map", 1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_creator");
  ros::NodeHandle nh;
  PathController pc;
  pc.init();

  ros::spin();
  return 0;
}