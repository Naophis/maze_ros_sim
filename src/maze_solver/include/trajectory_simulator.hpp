#ifndef TRAJECTORY_SIMULATOR_H
#define TRAJECTORY_SIMULATOR_H

#include "maze_solver.hpp"
#include "my_msg/base_path.h"
#include "my_msg/base_path_element.h"
#include "stdio.h"
#include "trajectory_creator.hpp"
#include <ros/ros.h>
#include <vector>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace visualization_msgs;

class TrajectorySimulator {
private:
  ros::Timer timer;
  ros::NodeHandle _nh;
  ros::Publisher pub_base_path;
  ros::Publisher pub_trajectory;
  ros::Publisher pub_trajectory_dia;
  ros::Publisher pub_test_trajectory;
  ros::Subscriber sub_base_path;
  ros::Subscriber sub_base_path_dia;

  MarkerArrayPtr trajectory_mk_array;
  MarkerArrayPtr test_trajectory_mk_array;

  MarkerPtr mk_trajectory;
  TrajectoryCreator tc;
  void make_all_trajectory();

public:
  TrajectorySimulator(/* args */);
  ~TrajectorySimulator();
  void init();
  void set_node_handler(const ros::NodeHandle &nh) { _nh = nh; }
  void timer_callback(const ros::TimerEvent &e);
  void base_path_callback(const my_msg::base_pathConstPtr &bp);
  void base_path_dia_callback(const my_msg::base_pathConstPtr &bp);
  void exec();
  void set_turn_param();
};

TrajectorySimulator::TrajectorySimulator(/* args */) {}

TrajectorySimulator::~TrajectorySimulator() {}

#endif