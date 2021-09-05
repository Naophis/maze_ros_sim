#include "trajectory_simulator.hpp"

void TrajectorySimulator::base_path_dia_callback(
    const my_msg::base_pathConstPtr &bp) {
  // return;
  path_struct base_path;
  path_element ele;
  for (const auto ph : bp->paths) {
    ele.s = ph.s;
    ele.t = ph.t;
    base_path.paths.emplace_back(ele);
  }
  base_path.size = bp->size;
  vector<trajectory_point_t> trajectory;
  tc.exec2(base_path, trajectory);

  trajectory_mk_array->markers.resize(1);
  trajectory_mk_array->markers[0].action = Marker::DELETEALL;

  visualization_msgs::Marker straight_line;
  straight_line.header.frame_id = "maze_base";
  straight_line.header.stamp = ros::Time::now();
  straight_line.ns = "points_and_straight_lines";
  straight_line.action = visualization_msgs::Marker::ADD;
  straight_line.pose.orientation.w = 0.000;
  straight_line.id = 2;
  straight_line.type = visualization_msgs::Marker::POINTS;
  straight_line.scale.x = 0.005;
  straight_line.scale.y = 0.005;
  straight_line.scale.z = 0.005;
  straight_line.color.r = 1;
  straight_line.color.g = 1;
  straight_line.color.b = 0.5;
  straight_line.color.a = 1;

  visualization_msgs::Marker front_str_line;
  front_str_line.header.frame_id = "maze_base";
  front_str_line.header.stamp = ros::Time::now();
  front_str_line.ns = "points_and_front_str_lines";
  front_str_line.action = visualization_msgs::Marker::ADD;
  front_str_line.pose.orientation.w = 0.000;
  front_str_line.id = 3;
  front_str_line.type = visualization_msgs::Marker::POINTS;
  front_str_line.scale.x = 0.005;
  front_str_line.scale.y = 0.005;
  front_str_line.scale.z = 0.005;
  front_str_line.color.r = 0.5;
  front_str_line.color.g = 0.5;
  front_str_line.color.b = 1;
  front_str_line.color.a = 1;

  visualization_msgs::Marker turn_line;
  turn_line.header.frame_id = "maze_base";
  turn_line.header.stamp = ros::Time::now();
  turn_line.ns = "points_and_turn_lines";
  turn_line.action = visualization_msgs::Marker::ADD;
  turn_line.pose.orientation.w = 0.000;
  turn_line.id = 4;
  turn_line.type = visualization_msgs::Marker::POINTS;
  turn_line.scale.x = 0.005;
  turn_line.scale.y = 0.005;
  turn_line.scale.z = 0.005;
  turn_line.color.r = 1;
  turn_line.color.g = 0.5;
  turn_line.color.b = 1;
  turn_line.color.a = 1;

  visualization_msgs::Marker back_str_line;
  back_str_line.header.frame_id = "maze_base";
  back_str_line.header.stamp = ros::Time::now();
  back_str_line.ns = "points_and_back_str_lines";
  back_str_line.action = visualization_msgs::Marker::ADD;
  back_str_line.pose.orientation.w = 0.000;
  back_str_line.id = 5;
  back_str_line.type = visualization_msgs::Marker::POINTS;
  back_str_line.scale.x = 0.005;
  back_str_line.scale.y = 0.005;
  back_str_line.scale.z = 0.005;
  back_str_line.color.r = 1;
  back_str_line.color.g = 0;
  back_str_line.color.b = 0;
  back_str_line.color.a = 1;

  for (const auto trj : trajectory) {
    geometry_msgs::Point p;
    p.x = trj.x / 1000;
    p.y = trj.y / 1000;
    p.z = 0.01;
    if (trj.type == 0) {
      straight_line.points.emplace_back(p);
    } else if (trj.type == 1) {
      front_str_line.points.emplace_back(p);
    } else if (trj.type == 2) {
      turn_line.points.emplace_back(p);
    } else if (trj.type == 3) {
      back_str_line.points.emplace_back(p);
    }
  }
  trajectory_mk_array->markers.emplace_back(straight_line);
  trajectory_mk_array->markers.emplace_back(front_str_line);
  trajectory_mk_array->markers.emplace_back(turn_line);
  trajectory_mk_array->markers.emplace_back(back_str_line);

  pub_trajectory_dia.publish(trajectory_mk_array);
}
void TrajectorySimulator::base_path_callback(
    const my_msg::base_pathConstPtr &bp) {}

void TrajectorySimulator::make_all_trajectory() {
  vector<trajectory_point_t> trajectory;
  tc.make_chopped_trajectory(trajectory, 3, false);
}

void TrajectorySimulator::timer_callback(const ros::TimerEvent &e) {
  //
  base_trajectory_pattern_t trajectory;
  tc.make_chopped_trajectory(trajectory.normal, 1, false);
  tc.make_chopped_trajectory(trajectory.orval, 3, false);
  tc.make_chopped_trajectory(trajectory.large, 5, false);
  tc.make_chopped_trajectory(trajectory.dia45, 7, false);
  tc.make_chopped_trajectory(trajectory.dia45_2, 7, true);
  tc.make_chopped_trajectory(trajectory.dia135, 9, false);
  tc.make_chopped_trajectory(trajectory.dia135_2, 9, true);
  tc.make_chopped_trajectory(trajectory.dia90, 11, true);

  ROS_WARN("%ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld, ", trajectory.normal.size(),
           trajectory.orval.size(), trajectory.large.size(),
           trajectory.dia45.size(), trajectory.dia45_2.size(),
           trajectory.dia135.size(), trajectory.dia135_2.size(),
           trajectory.dia90.size());

  test_trajectory_mk_array->markers.resize(1);
  test_trajectory_mk_array->markers[0].action = Marker::DELETEALL;

  visualization_msgs::Marker front_str_line;
  front_str_line.header.frame_id = "maze_base";
  front_str_line.header.stamp = ros::Time::now();
  front_str_line.ns = "points_and_front_str_lines";
  front_str_line.action = visualization_msgs::Marker::ADD;
  front_str_line.pose.orientation.w = 0.000;
  front_str_line.id = 3;
  front_str_line.type = visualization_msgs::Marker::POINTS;
  front_str_line.scale.x = 0.005;
  front_str_line.scale.y = 0.005;
  front_str_line.scale.z = 0.005;
  front_str_line.color.r = 0.5;
  front_str_line.color.g = 0.5;
  front_str_line.color.b = 1;
  front_str_line.color.a = 1;

  visualization_msgs::Marker turn_line;
  turn_line.header.frame_id = "maze_base";
  turn_line.header.stamp = ros::Time::now();
  turn_line.ns = "points_and_turn_lines";
  turn_line.action = visualization_msgs::Marker::ADD;
  turn_line.pose.orientation.w = 0.000;
  turn_line.id = 4;
  turn_line.type = visualization_msgs::Marker::POINTS;
  turn_line.scale.x = 0.005;
  turn_line.scale.y = 0.005;
  turn_line.scale.z = 0.005;
  turn_line.color.r = 1;
  turn_line.color.g = 0.5;
  turn_line.color.b = 1;
  turn_line.color.a = 1;

  visualization_msgs::Marker back_str_line;
  back_str_line.header.frame_id = "maze_base";
  back_str_line.header.stamp = ros::Time::now();
  back_str_line.ns = "points_and_back_str_lines";
  back_str_line.action = visualization_msgs::Marker::ADD;
  back_str_line.pose.orientation.w = 0.000;
  back_str_line.id = 5;
  back_str_line.type = visualization_msgs::Marker::POINTS;
  back_str_line.scale.x = 0.005;
  back_str_line.scale.y = 0.005;
  back_str_line.scale.z = 0.005;
  back_str_line.color.r = 1;
  back_str_line.color.g = 0;
  back_str_line.color.b = 0;
  back_str_line.color.a = 1;

  for (const auto trj : trajectory.dia135) {
    geometry_msgs::Point p;
    p.x = trj.x / 1000;
    p.y = trj.y / 1000;
    p.z = 0.01;
    if (trj.type == 1) {
      front_str_line.points.emplace_back(p);
    } else if (trj.type == 2) {
      turn_line.points.emplace_back(p);
    } else if (trj.type == 3) {
      back_str_line.points.emplace_back(p);
    }
  }
  test_trajectory_mk_array->markers.emplace_back(front_str_line);
  test_trajectory_mk_array->markers.emplace_back(turn_line);
  test_trajectory_mk_array->markers.emplace_back(back_str_line);
  pub_test_trajectory.publish(test_trajectory_mk_array);
}

void TrajectorySimulator::set_turn_param() {
  XmlRpc::XmlRpcValue prm;
  _nh.getParam("/slalom", prm);
  tc.sla_data = {0};

  tc.sla_data.normal.radius = static_cast<double>(prm["normal"]["radius"]);
  tc.sla_data.normal.time = static_cast<double>(prm["normal"]["time"]);
  tc.sla_data.normal.n = static_cast<double>(prm["normal"]["n"]);
  tc.sla_data.normal.front = static_cast<double>(prm["normal"]["front"]);
  tc.sla_data.normal.back = static_cast<double>(prm["normal"]["back"]);

  tc.sla_data.large.radius = static_cast<double>(prm["large"]["radius"]);
  tc.sla_data.large.time = static_cast<double>(prm["large"]["time"]);
  tc.sla_data.large.n = static_cast<double>(prm["large"]["n"]);
  tc.sla_data.large.front = static_cast<double>(prm["large"]["front"]);
  tc.sla_data.large.back = static_cast<double>(prm["large"]["back"]);

  tc.sla_data.orval.radius = static_cast<double>(prm["orval"]["radius"]);
  tc.sla_data.orval.time = static_cast<double>(prm["orval"]["time"]);
  tc.sla_data.orval.n = static_cast<double>(prm["orval"]["n"]);
  tc.sla_data.orval.front = static_cast<double>(prm["orval"]["front"]);
  tc.sla_data.orval.back = static_cast<double>(prm["orval"]["back"]);

  tc.sla_data.dia45.radius = static_cast<double>(prm["dia45"]["radius"]);
  tc.sla_data.dia45.time = static_cast<double>(prm["dia45"]["time"]);
  tc.sla_data.dia45.n = static_cast<double>(prm["dia45"]["n"]);
  tc.sla_data.dia45.front = static_cast<double>(prm["dia45"]["front"]);
  tc.sla_data.dia45.back = static_cast<double>(prm["dia45"]["back"]);

  tc.sla_data.dia45_2.radius = static_cast<double>(prm["dia45_2"]["radius"]);
  tc.sla_data.dia45_2.time = static_cast<double>(prm["dia45_2"]["time"]);
  tc.sla_data.dia45_2.n = static_cast<double>(prm["dia45_2"]["n"]);
  tc.sla_data.dia45_2.front = static_cast<double>(prm["dia45_2"]["front"]);
  tc.sla_data.dia45_2.back = static_cast<double>(prm["dia45_2"]["back"]);

  tc.sla_data.dia135.radius = static_cast<double>(prm["dia135"]["radius"]);
  tc.sla_data.dia135.time = static_cast<double>(prm["dia135"]["time"]);
  tc.sla_data.dia135.n = static_cast<double>(prm["dia135"]["n"]);
  tc.sla_data.dia135.front = static_cast<double>(prm["dia135"]["front"]);
  tc.sla_data.dia135.back = static_cast<double>(prm["dia135"]["back"]);

  tc.sla_data.dia135_2.radius = static_cast<double>(prm["dia135_2"]["radius"]);
  tc.sla_data.dia135_2.time = static_cast<double>(prm["dia135_2"]["time"]);
  tc.sla_data.dia135_2.n = static_cast<double>(prm["dia135_2"]["n"]);
  tc.sla_data.dia135_2.front = static_cast<double>(prm["dia135_2"]["front"]);
  tc.sla_data.dia135_2.back = static_cast<double>(prm["dia135_2"]["back"]);

  tc.sla_data.dia90.radius = static_cast<double>(prm["dia90"]["radius"]);
  tc.sla_data.dia90.time = static_cast<double>(prm["dia90"]["time"]);
  tc.sla_data.dia90.n = static_cast<double>(prm["dia90"]["n"]);
  tc.sla_data.dia90.front = static_cast<double>(prm["dia90"]["front"]);
  tc.sla_data.dia90.back = static_cast<double>(prm["dia90"]["back"]);

  tc.init();
}

void TrajectorySimulator::init() {
  set_turn_param();

  // timer = _nh.createTimer(ros::Duration(0.2),
  //                         &TrajectorySimulator::timer_callback, this);
  // sub_base_path = _nh.subscribe("/base_path", 10,
  //                               &TrajectorySimulator::base_path_callback,
  //                               this);

  sub_base_path_dia = _nh.subscribe(
      "/base_path_dia", 10, &TrajectorySimulator::base_path_dia_callback, this);
  // pub_base_path = _nh.advertise<my_msg::base_path>("/base_path", 1);

  pub_trajectory = _nh.advertise<Marker>("/viewer/base_path", 1);
  trajectory_mk_array.reset(new MarkerArray);
  test_trajectory_mk_array.reset(new MarkerArray);
  pub_trajectory_dia = _nh.advertise<MarkerArray>("/viewer/base_path_dia", 1);
  pub_test_trajectory = _nh.advertise<MarkerArray>("/viewer/base_path_dia2", 1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_simulator");
  ros::NodeHandle nh;

  TrajectorySimulator ts;
  ts.set_node_handler(nh);
  ts.init();
  ros::spin();
  return 0;
}