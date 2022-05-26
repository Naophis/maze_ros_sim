
#include <ros/ros.h>

#include "controller.hpp"

void MazeSolverCtrl::update_sensing_result() {
  int x = ego.x;
  int y = ego.y;
  char north_wall = existWall_from_premap(x, y, Direction::North);
  char east_wall = existWall_from_premap(x, y, Direction::East);
  char west_wall = existWall_from_premap(x, y, Direction::West);
  char south_wall = existWall_from_premap(x, y, Direction::South);

  lgc.set_wall_data(x, y, Direction::North, north_wall);
  lgc.set_wall_data(x, y + 1, Direction::South, north_wall);

  lgc.set_wall_data(x, y, Direction::East, east_wall);
  lgc.set_wall_data(x + 1, y, Direction::West, east_wall);

  lgc.set_wall_data(x, y, Direction::West, west_wall);
  lgc.set_wall_data(x - 1, y, Direction::East, west_wall);

  lgc.set_wall_data(x, y, Direction::South, south_wall);
  lgc.set_wall_data(x, y - 1, Direction::North, south_wall);
}

void MazeSolverCtrl::update_sensing_result2() {
  for (int x = 0; x < maze_size; x++) {
    for (int y = 0; y < maze_size; y++) {
      char north_wall = existWall_from_premap(x, y, Direction::North);
      char east_wall = existWall_from_premap(x, y, Direction::East);
      char west_wall = existWall_from_premap(x, y, Direction::West);
      char south_wall = existWall_from_premap(x, y, Direction::South);

      lgc.set_wall_data(x, y, Direction::North, north_wall);
      lgc.set_wall_data(x, y + 1, Direction::South, north_wall);

      lgc.set_wall_data(x, y, Direction::East, east_wall);
      lgc.set_wall_data(x + 1, y, Direction::West, east_wall);

      lgc.set_wall_data(x, y, Direction::West, west_wall);
      lgc.set_wall_data(x - 1, y, Direction::East, west_wall);

      lgc.set_wall_data(x, y, Direction::South, south_wall);
      lgc.set_wall_data(x, y - 1, Direction::North, south_wall);
    }
  }
  for (int x = 0; x < maze_size; x++)
    for (int y = 0; y < maze_size; y++)
      adachi.deadEnd(x, y);
}

bool MazeSolverCtrl::existWall_from_premap(int x, int y, Direction dir) {
  if (x < 0 || x >= maze_size || y < 0 || y >= maze_size)
    return true;
  return ((pre_map_data[x][y] / static_cast<int>(dir)) & 0x01) == 0x01;
}

void MazeSolverCtrl::timer_callback(const ros::TimerEvent &e) {
  my_msg::maze mz;
  mz.header.stamp = ros::Time::now();

  global_cnt++;

  update_sensing_result();
  // mz.dia_dist_n.clear();
  // mz.dia_dist_e.clear();
  mz.dia_dist_n.resize(maze_size * maze_size);
  mz.dia_dist_e.resize(maze_size * maze_size);

  mz.maze_size = maze_size;
  path_type path;

  if (maze_known) {
    update_sensing_result2();
    maze_known = false;
  }
  mz.motion = static_cast<int>(adachi.exec(path));
  Motion next_motion = static_cast<Motion>(mz.motion);
  if (adachi.goal_step) {
    adachi.subgoal_list.erase(ego.x + ego.y * lgc.maze_size);
    {
      lgc.set_param3();
      lgc.searchGoalPosition(true, adachi.subgoal_list);
      adachi.cost_mode = 3;
    }
  }
  {
    my_msg::base_path bp;
    my_msg::base_path_element ele;
    bp.paths.clear();
    bp.size = 0;
    if (next_motion == Motion::Straight) {
      ele.s = 2;
      ele.t = static_cast<int>(PathMotion::End);
      bp.paths.emplace_back(ele);
    } else if (next_motion == Motion::TurnRight) {
      ele.s = 0;
      ele.t = static_cast<int>(PathMotion::Right);
      bp.paths.emplace_back(ele);
    } else if (next_motion == Motion::TurnLeft) {
      ele.s = 0;
      ele.t = static_cast<int>(PathMotion::Left);
      bp.paths.emplace_back(ele);
    } else if (next_motion == Motion::Back) {
      ele.s = 1;
      ele.t = static_cast<int>(PathMotion::Pivot180);
      bp.paths.emplace_back(ele);
      ele.s = 1;
      ele.t = static_cast<int>(PathMotion::End);
      bp.paths.emplace_back(ele);
    }
    bp.size = bp.paths.size();
    pub_base_path.publish(bp);
  }

  mz.cost_mode = adachi.cost_mode;

  for (int i = 0; i < maze_size; i++) {
    for (int j = 0; j < maze_size; j++) {
      mz.wall.emplace_back(lgc.get_map_val(i, j));
      mz.dist.emplace_back(lgc.get_dist_val(i, j));
      mz.dia_dist_n[i + j * maze_size] = lgc.get_diadist_n_val(i, j);
      mz.dia_dist_e[i + j * maze_size] = lgc.get_diadist_e_val(i, j);
      // mz.dia_dist_n.emplace_back(lgc.get_diadist_n_val(i, j));
      // mz.dia_dist_e.emplace_back(lgc.get_diadist_e_val(i, j));
    }
  }

  mz.ego.pos_x = adachi.ego->x;
  mz.ego.pos_y = adachi.ego->y;
  mz.ego.dir = static_cast<int>(adachi.ego->dir);

  pub_maze_data.publish(mz);
}

void MazeSolverCtrl::init() {
  set_meta_maze_data();
  adachi.set_ego(&ego);
  adachi.set_logic(&lgc);
  lgc.set_ego(&ego);
  adachi.lgc->set_param3();

  lgc.set_default_wall_data();
  ego.x = 0;
  ego.y = 0;
  ego.dir = Direction::North;
  update_sensing_result();
  ego.y++;

  timer = _nh.createTimer(ros::Duration(0.1), &MazeSolverCtrl::timer_callback,
                          this);
  pub_maze_data = _nh.advertise<my_msg::maze>("/maze", 1);
  pub_base_path = _nh.advertise<my_msg::base_path>("/base_path", 1);
}
void MazeSolverCtrl::set_meta_maze_data() {
  XmlRpc::XmlRpcValue prm;

  _nh.getParam("/maze_data", prm);

  if (!(prm["wall"].valid() && prm["maze_size"].valid())) {
    ROS_WARN("rosparam /maze_data is not found");
    return;
  }
  if (prm["known"].valid()) {
    maze_known = static_cast<int>(prm["known"]) > 0;
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