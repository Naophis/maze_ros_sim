#ifndef PATH_CONTROLLER_H
#define PATH_CONTROLLER_H

#include <ros/ros.h>

#include "maze_solver.hpp"
#include "my_msg/base_path.h"
#include "my_msg/base_path_element.h"
#include "my_msg/dist_map.h"
#include "my_msg/dist_map_q.h"
#include "my_msg/maze.h"
#include "my_msg/path.h"

#include "my_msg/candidate_cell_set.h"
#include "my_msg/candidate_route_map.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "path_creator.hpp"

using namespace std;
using namespace visualization_msgs;

class PathController {
private:
  ros::NodeHandle _nh;

  vector<point_t> goal_list;
  point_t tmp_goal_p;

  ros::Publisher pub_path;
  ros::Publisher pub_path_dia;
  ros::Publisher pub_base_path_dia;
  ros::Publisher pub_dist_map_q;
  ros::Publisher pub_candidate_map;
  ros::Subscriber sub_maze;

  PathCreator pc;
  int maze_size;
  int max_step_val;
  void set_meta_data();
  void maze_callback(const my_msg::mazeConstPtr &_mz);

public:
  PathController(/* args */);
  ~PathController();

  void init();
  void set_node_handler(const ros::NodeHandle &nh) { _nh = nh; }
};

#endif