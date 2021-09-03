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

  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "maze_base";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "points_and_lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 0.000;
  line_list.id = 2;
  line_list.type = visualization_msgs::Marker::POINTS;
  line_list.scale.x = 0.005;
  line_list.scale.y = 0.005;
  line_list.scale.z = 0.005;
  line_list.color.r = 1;
  line_list.color.g = 1;
  line_list.color.b = 0.5;
  line_list.color.a = 1;
  for (const auto trj : trajectory) {
    geometry_msgs::Point p;
    p.x = trj.x / 1000;
    p.y = trj.y / 1000;
    p.z = 0.01;
    line_list.points.emplace_back(p);
  }
  pub_trajectory_dia.publish(line_list);
}
void TrajectorySimulator::base_path_callback(
    const my_msg::base_pathConstPtr &bp) {
  // path_struct base_path;
  // path_element ele;
  // for (const auto ph : bp->paths) {
  //   ele.s = ph.s;
  //   ele.t = ph.t;
  //   base_path.paths.emplace_back(ele);
  // }
  // base_path.size = bp->size;
  // vector<trajectory_point_t> trajectory;
  // tc.exec(base_path, trajectory);

  // visualization_msgs::Marker line_list;
  // line_list.header.frame_id = "ego_trajectory";
  // line_list.header.stamp = ros::Time::now();
  // line_list.ns = "points_and_lines";
  // line_list.action = visualization_msgs::Marker::ADD;
  // line_list.pose.orientation.w = 0.000;
  // line_list.id = 2;
  // line_list.type = visualization_msgs::Marker::POINTS;
  // line_list.scale.x = 0.005;
  // line_list.scale.y = 0.005;
  // line_list.scale.z = 0.005;
  // line_list.color.r = 1;
  // line_list.color.g = 1;
  // line_list.color.b = 0.5;
  // line_list.color.a = 1;
  // for (const auto trj : trajectory) {
  //   geometry_msgs::Point p;
  //   p.x = trj.x / 1000;
  //   p.y = trj.y / 1000;
  //   p.z = 0.01;
  //   line_list.points.emplace_back(p);
  // }
  // pub_trajectory.publish(line_list);
}

void TrajectorySimulator::timer_callback(const ros::TimerEvent &e) {
  // my_msg::base_path bp;
  // my_msg::base_path_element ele;
  // Motion next_motion = Motion::TurnRight;
  // {
  //   bp.paths.clear();
  //   bp.size = 0;
  //   if (next_motion == Motion::Straight) {
  //     ele.s = 2;
  //     ele.t = static_cast<int>(PathMotion::End);
  //     bp.paths.emplace_back(ele);
  //   } else if (next_motion == Motion::TurnRight) {
  //     ele.s = 0;
  //     ele.t = static_cast<int>(PathMotion::Right);
  //     bp.paths.emplace_back(ele);
  //   } else if (next_motion == Motion::TurnLeft) {
  //     ele.s = 0;
  //     ele.t = static_cast<int>(PathMotion::Left);
  //     bp.paths.emplace_back(ele);
  //   } else if (next_motion == Motion::Back) {
  //     ele.s = 1;
  //     ele.t = static_cast<int>(PathMotion::Pivot180);
  //     bp.paths.emplace_back(ele);
  //     ele.s = 1;
  //     ele.t = static_cast<int>(PathMotion::End);
  //     bp.paths.emplace_back(ele);
  //   }
  //   bp.size = bp.paths.size();
  // }

  // pub_base_path.publish(bp);
}

void TrajectorySimulator::init() {
  // timer = _nh.createTimer(ros::Duration(0.1),
  //                         &TrajectorySimulator::timer_callback, this);

  sub_base_path = _nh.subscribe("/base_path", 10,
                                &TrajectorySimulator::base_path_callback, this);

  sub_base_path_dia = _nh.subscribe(
      "/base_path_dia", 10, &TrajectorySimulator::base_path_dia_callback, this);
  // pub_base_path = _nh.advertise<my_msg::base_path>("/base_path", 1);

  pub_trajectory = _nh.advertise<Marker>("/viewer/base_path", 1);
  pub_trajectory_dia = _nh.advertise<Marker>("/viewer/base_path_dia", 1);
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