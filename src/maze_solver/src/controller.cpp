
#include <ros/ros.h>

#include "controller.hpp"

void MazeSolverCtrl::update_sensing_result() {
  int x = ego.x;
  int y = ego.y;
  int dir = ego.dir;
  char north_wall = existWall_from_premap(x, y, North);
  char east_wall = existWall_from_premap(x, y, East);
  char west_wall = existWall_from_premap(x, y, West);
  char south_wall = existWall_from_premap(x, y, South);

  lgc.set_wall_data(x, y, North, north_wall);
  lgc.set_wall_data(x, y + 1, South, north_wall);

  lgc.set_wall_data(x, y, East, east_wall);
  lgc.set_wall_data(x + 1, y, West, east_wall);

  lgc.set_wall_data(x, y, West, west_wall);
  lgc.set_wall_data(x - 1, y, East, west_wall);

  lgc.set_wall_data(x, y, South, south_wall);
  lgc.set_wall_data(x, y - 1, North, south_wall);
}

bool MazeSolverCtrl::existWall_from_premap(int x, int y, int dir) {
  if (x < 0 || x >= maze_size || y < 0 || y >= maze_size)
    return true;
  return ((pre_map_data[x][y] / dir) & 0x01) == 0x01;
}

void MazeSolverCtrl::timer_callback(const ros::TimerEvent &e) {
  my_msg::maze mz;
  mz.header.stamp = ros::Time::now();

  global_cnt++;

  update_sensing_result();

  mz.dia_dist_n.resize(maze_size * maze_size);
  mz.dia_dist_e.resize(maze_size * maze_size);

  mz.maze_size = maze_size;
  mz.motion = adachi.exec();
  mz.cost_mode = adachi.cost_mode;

  for (int i = 0; i < maze_size; i++) {
    for (int j = 0; j < maze_size; j++) {
      mz.wall.push_back(lgc.get_map_val(i, j));
      mz.dist.push_back(lgc.get_dist_val(i, j));
      mz.dia_dist_n[i + j * maze_size] = lgc.get_diadist_n_val(i, j);
      mz.dia_dist_e[i + j * maze_size] = lgc.get_diadist_e_val(i, j);
    }
  }

  mz.ego.pos_x = adachi.ego->x;
  mz.ego.pos_y = adachi.ego->y;
  mz.ego.dir = adachi.ego->dir;

  pub_maze_data.publish(mz);
}

void MazeSolverCtrl::init() {
  set_meta_maze_data();
  adachi.set_ego(&ego);
  adachi.set_logic(&lgc);
  adachi.lgc->set_param3();

  lgc.set_default_wall_data();
  ego.x = 0;
  ego.y = 0;
  ego.dir = North;
  update_sensing_result();
  ego.y++;

  timer = _nh.createTimer(ros::Duration(0.1), &MazeSolverCtrl::timer_callback,
                          this);
  pub_maze_data = _nh.advertise<my_msg::maze>("/maze", 1);
}
void MazeSolverCtrl::set_meta_maze_data() {
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
    goal_list.push_back(tmp_goal_p);
  }
  int c = 0;

  for (int i = 0; i < maze_size; i++) {
    for (int j = 0; j < maze_size; j++) {
      pre_map_data[i][j] = static_cast<int>(prm["wall"][c]);
      dist[i][j] = max_step_val;
      c++;
    }
  }

  lgc.init(maze_size, max_step_val);
  lgc.set_goal_pos(goal_list);

  // for (int i = 0; i < maze_size; i++)
  //   for (int j = 0; j < maze_size; j++)
  //     lgc.set_map_val(i, j, map[i][j]);
}
void MazeSolverCtrl::update_step_map() {
  for (int i = 0; i < maze_size; i++)
    for (int j = 0; j < maze_size; j++)
      dist[i][j] = max_step_val;

  for (const auto p : goal_list)
    dist[p.x][p.y] = 0;
}

MazeSolverCtrl::MazeSolverCtrl(/* args */) {}

MazeSolverCtrl::~MazeSolverCtrl() {}

int main(int argc, char **argv) {
  ros::init(argc, argv, "maze_solver");
  ros::NodeHandle nh;
  MazeSolverCtrl ms;
  ms.set_node_handler(nh);
  ms.init();
  ros::spin();
  return 0;
}