#include "viewer.hpp"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

void Viewer::timer_callback(const ros::TimerEvent &e) {
  craete_wall();

  create_maze_info();
  pub_maze_info.publish(maze_info_list[loop]);

  if (initflag && mz.dia_dist_n.size() > 0) {
    create_maze_diainfo();
    pub_maze_diainfo.publish(maze_diainfo_list[loop]);
  }

  // create_ego_marker();
  create_ego_marker2();
  pub_ego_marker.publish(ego_marker_list[loop]);

  if (initflag2) {
    int path_size = ph.path_s.size();

    int dir = 1;
    double x = cell_size / 2;
    double y = cell_size;

    float staright = 0;

    visualization_msgs::Marker line_list;

    line_list.header.frame_id = "map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "points_and_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 0.2;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.025;
    line_list.scale.y = 0.025;
    line_list.scale.z = 0.025;

    // Line list is red

    float f = 0.0;
    const double z = -0.001;
    double alpha = 0.65;

    line_list.color.r = 0.5;
    line_list.color.g = 0.5;
    line_list.color.b = 0.5;

    if (mz.motion == 0) {
      // alpha = 1;
      line_list.color.r = 0.65;
      line_list.color.g = 0.65;
      line_list.color.b = 0.65;
    }

    line_list.color.a = alpha;
    for (int i = 0; i < path_size; i++) {
      double straight = ph.path_s[i] * 0.5 - 1;
      int turn = ph.path_t[i];
      if (i == 0)
        straight = (ph.path_s[i] - 2) * 0.5 - 0.5;
      straight = straight >= 0 ? straight : 0;

      if (straight > 0) {
        geometry_msgs::Point p;
        p.x = x / 1000;
        p.y = y / 1000;
        p.z = z;
        line_list.points.emplace_back(p);
        if (dir == 1) {
          y += straight * cell_size;
        } else if (dir == 2) {
          x += straight * cell_size;
        } else if (dir == 4) {
          x -= straight * cell_size;
        } else if (dir == 8) {
          y -= straight * cell_size;
        }
        p.x = x / 1000;
        p.y = y / 1000;
        p.z = z;
        line_list.points.emplace_back(p);
      }
      if (turn != 255) {
        geometry_msgs::Point p;
        p.x = x / 1000;
        p.y = y / 1000;
        p.z = z;
        line_list.points.emplace_back(p);
        if (dir == 1) {
          if (turn == 1) {
            x += cell_size / 2;
            y += cell_size / 2;
            dir = 2;
          } else if (turn == 2) {
            x -= cell_size / 2;
            y += cell_size / 2;
            dir = 4;
          }
        } else if (dir == 2) {
          if (turn == 1) {
            x += cell_size / 2;
            y -= cell_size / 2;
            dir = 8;
          } else if (turn == 2) {
            x += cell_size / 2;
            y += cell_size / 2;
            dir = 1;
          }
        } else if (dir == 4) {
          if (turn == 1) {
            x -= cell_size / 2;
            y += cell_size / 2;
            dir = 1;
          } else if (turn == 2) {
            x -= cell_size / 2;
            y -= cell_size / 2;
            dir = 8;
          }
        } else if (dir == 8) {
          if (turn == 1) {
            x -= cell_size / 2;
            y -= cell_size / 2;
            dir = 4;
          } else if (turn == 2) {
            x += cell_size / 2;
            y -= cell_size / 2;
            dir = 2;
          }
        }
        p.x = x / 1000;
        p.y = y / 1000;
        p.z = z;
        line_list.points.emplace_back(p);
      }
    }
    pub_path_marker.publish(line_list);
  }
  loop++;
  if (loop >= MAX_LOOP)
    loop = 0;
}

void Viewer::create_ego_marker() {
  ego_marker_list[loop]->markers.resize(1);
  ego_marker_list[loop]->markers[0].action = Marker::DELETEALL;

  Marker mk;
  mk.id = 0;
  mk.ns = "map";
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.type = Marker::ARROW;
  mk.action = Marker::ADD;
  mk.color.r = 0;
  mk.color.g = 1;
  mk.color.b = 1;
  mk.color.a = 1;

  double x1 = cell_size / 2 + cell_size * ego.x;
  double y1 = cell_size / 2 + cell_size * ego.y;
  double ang = 0;
  if (ego_dir == 1) {
    ang = PI / 2;
  } else if (ego_dir == 2) {
    ang = 0;
  } else if (ego_dir == 4) {
    ang = PI;
  } else if (ego_dir == 8) {
    ang = -PI / 2;
  }

  tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, ang);
  mk.pose.position.x = (x1) / 1000;
  mk.pose.position.y = (y1) / 1000;
  mk.pose.position.z = 0.1;
  mk.pose.orientation.x = quat[0];
  mk.pose.orientation.y = quat[1];
  mk.pose.orientation.z = quat[2];
  mk.pose.orientation.w = quat[3];
  mk.scale.x = 0.05;
  mk.scale.y = 0.05;
  mk.scale.z = 0.05;
  ego_marker_list[loop]->markers.emplace_back(mk);
}

void Viewer::create_ego_marker2() {
  ego_marker_list[loop]->markers.resize(1);
  ego_marker_list[loop]->markers[0].action = Marker::DELETEALL;

  Marker mk;
  mk.id = 0;
  mk.ns = "map";
  mk.header.frame_id = "ego";
  mk.header.stamp = ros::Time::now();
  mk.type = Marker::ARROW;
  mk.action = Marker::ADD;
  mk.color.r = 0;
  mk.color.g = 1;
  mk.color.b = 1;
  mk.color.a = 1;
  tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, 0);
  mk.pose.position.x = 0;
  mk.pose.position.y = 0;
  mk.pose.position.z = 0;
  mk.pose.orientation.x = quat[0];
  mk.pose.orientation.y = quat[1];
  mk.pose.orientation.z = quat[2];
  mk.pose.orientation.w = quat[3];
  mk.scale.x = 0.05;
  mk.scale.y = 0.05;
  mk.scale.z = 0.05;
  ego_marker_list[loop]->markers.emplace_back(mk);

  double x1 = cell_size / 2 + cell_size * ego.x;
  double y1 = cell_size / 2 + cell_size * ego.y;
  double ang = 0;

  if (ego_dir == 1) {
    ang = PI / 2;
    y1 -= cell_size / 2;
  } else if (ego_dir == 2) {
    ang = 0;
    x1 -= cell_size / 2;
  } else if (ego_dir == 4) {
    ang = PI;
    x1 += cell_size / 2;
  } else if (ego_dir == 8) {
    ang = -PI / 2;
    y1 += cell_size / 2;
  }

  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "ego";
  transformStamped.transform.translation.x = x1 / 1000;
  transformStamped.transform.translation.y = y1 / 1000;
  transformStamped.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, ang);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  dynamic_br_.sendTransform(transformStamped);
}

void Viewer::create_maze_diainfo() {
  maze_diainfo_list[loop]->markers.resize(1);
  maze_diainfo_list[loop]->markers[0].action = Marker::DELETEALL;

  if (!initflag)
    return;
  int id = 0;
  for (int x = 0; x < maze_size; x++) {
    for (int y = 0; y < maze_size; y++) {
      if (x < maze_size && (map[x][y] & 0x02) == 0x00)
        create_maze_info2(x, y, id, 2);
      if (y < maze_size && (map[x][y] & 0x01) == 0x00)
        create_maze_info2(x, y, id, 1);
    }
  }
}

void Viewer::create_maze_info2(double x, double y, int &id, int dir) {
  double x1 = cell_size / 2 + cell_size * x;
  double y1 = cell_size / 2 + cell_size * y;
  double x2 = 0, y2 = 0;
  float val = 0;
  int xx = x;
  int yy = y;
  // ROS_WARN("%d, %d", xx, yy);
  int size = mz.dia_dist_n.size();
  if (size == 0)
    return;
  if (dir == 0x01) {
    y2 = cell_size / 2;
    val = mz.dia_dist_n[xx + yy * maze_size];
  } else if (dir == 0x02) {
    x2 = cell_size / 2;
    val = mz.dia_dist_e[xx + yy * maze_size];
  }
  if (val > 180 * 1000) {
    return;
  }
  if ((map[xx][yy] & 0xf0) == 0x00) {
    return;
  }

  tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, 0);
  Marker mk;
  mk.id = id++;
  mk.ns = "map";
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.type = Marker::TEXT_VIEW_FACING;
  mk.action = Marker::ADD;
  mk.color.r = 0;
  mk.color.g = 0.5;
  mk.color.b = 0.5;
  mk.color.a = 1;

  if (dir == 1)
    if ((map[xx][yy] & 0x10) == 0x10) {
      mk.color.r = 0;
      mk.color.g = 1;
      mk.color.b = 1;
      mk.color.a = 1;
    }
  if (dir == 2)
    if ((map[xx][yy] & 0x20) == 0x20) {
      mk.color.r = 0;
      mk.color.g = 1;
      mk.color.b = 1;
      mk.color.a = 1;
    }
  mk.pose.position.x = (x1 + x2) / 1000;
  mk.pose.position.y = (y1 + y2) / 1000;
  mk.pose.position.z = 0.07;
  mk.pose.position.z = -0.001;
  mk.pose.orientation.x = quat[0];
  mk.pose.orientation.y = quat[1];
  mk.pose.orientation.z = quat[2];
  mk.pose.orientation.w = quat[3];
  mk.scale.z = 0.03;
  // if (dir == 1)
  //   snprintf(buf, TXT_BUF, "(%d, %d).n\n%d", (int)x, (int)y, val);
  // else
  //   snprintf(buf, TXT_BUF, "(%d, %d).e\n%d", (int)x, (int)y, val);
  if (val > 180 * 1000) {
    mk.text = "-";
  } else {
    snprintf(buf, TXT_BUF, "%0.2f", val);
    mk.text = &buf[0];
  }
  maze_diainfo_list[loop]->markers.emplace_back(mk);
}

void Viewer::create_maze_info() {
  maze_info_list[loop]->markers.resize(1);
  maze_info_list[loop]->markers[0].action = Marker::DELETEALL;
  int id = 0;
  for (int x = 0; x < maze_size; x++) {
    for (int y = 0; y < maze_size; y++) {
      double x1 = cell_size / 2 + cell_size * x;
      double y1 = cell_size / 2 + cell_size * y;
      double x2 = 0, y2 = 0;
      if (dist[x][y] > 1000) {
        continue;
      }
      if ((map[x][y] & 0xf0) == 0x00 && dist[x][y] != 0) {
        continue;
      }

      tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, 0);
      Marker mk;
      mk.id = id++;
      mk.ns = "map";
      mk.header.frame_id = "map";
      mk.header.stamp = ros::Time::now();
      mk.type = Marker::TEXT_VIEW_FACING;
      mk.action = Marker::ADD;

      mk.color.r = 0;
      mk.color.g = 0.5;
      mk.color.b = 0;
      mk.color.a = 1;
      mk.scale.z = 0.05;

      if ((map[x][y] & 0xf0) == 0xf0) {
        mk.color.r = 0;
        mk.color.g = 1;
        mk.color.b = 0;
        mk.color.a = 1;
      }
      if (dist[x][y] == 0) {
        mk.color.r = 1;
        mk.color.g = 1;
        mk.color.b = 0;
        mk.scale.z = 0.1;
      }

      mk.pose.position.x = (x1 + x2) / 1000;
      mk.pose.position.y = (y1 + y2) / 1000;
      mk.pose.position.z = -0.001;
      mk.pose.orientation.x = quat[0];
      mk.pose.orientation.y = quat[1];
      mk.pose.orientation.z = quat[2];
      mk.pose.orientation.w = quat[3];

      snprintf(buf, TXT_BUF, "%d", dist[x][y]);
      if (dist[x][y] == 0) {
        mk.text = "G";
      } else {
        mk.text = &buf[0];
      }
      maze_info_list[loop]->markers.emplace_back(mk);
      if (x < maze_size && (map[x][y] & 0x02) == 0x00)
        create_maze_info2(x, y, id, 2);
      if (y < maze_size && (map[x][y] & 0x01) == 0x00)
        create_maze_info2(x, y, id, 1);
    }
  }
}
void Viewer::init() {
  set_map();

  im_server.reset(
      new interactive_markers::InteractiveMarkerServer("viewer", "", false));
  ros::Duration(0.1).sleep();

  tf::Vector3 position;
  double x = cell_size * 8 / 1000;
  double y = cell_size * 8 / 1000;
  double z = -0.1;
  position = tf::Vector3(x, y, z);
  makeButtonMarker(position);

  im_server->applyChanges();

  timer = _nh.createTimer(ros::Duration(0.1), &Viewer::timer_callback, this);
  for (int i = 0; i < MAX_LOOP; i++) {
    wall_list[i].reset(new MarkerArray);
    maze_info_list[i].reset(new MarkerArray);
    maze_diainfo_list[i].reset(new MarkerArray);
    ego_marker_list[i].reset(new MarkerArray);
    path_array_list[i].reset(new Marker);
  }
  pub_wall_list[0] = _nh.advertise<MarkerArray>("/viewer/wall_list0", 1);
  pub_wall_list[1] = _nh.advertise<MarkerArray>("/viewer/wall_list1", 1);
  pub_wall_list[2] = _nh.advertise<MarkerArray>("/viewer/wall_list2", 1);
  pub_wall_list[3] = _nh.advertise<MarkerArray>("/viewer/wall_list3", 1);
  pub_text1 = _nh.advertise<OverlayText>("/viewer/text1", 1);
  pub_maze_info = _nh.advertise<MarkerArray>("/viewer/maze_info_list", 1);
  pub_maze_diainfo = _nh.advertise<MarkerArray>("/viewer/maze_diainfo_list", 1);
  pub_ego_marker = _nh.advertise<MarkerArray>("/viewer/ego_marker", 1);

  pub_path_marker = _nh.advertise<Marker>("/viewer/path_marker", 1);

  for (int i = 0; i < 4; i++) {
    wall_list[0].reset(new MarkerArray);
    wall_list[0]->markers.resize(1);
    wall_list[0]->markers[0].action = Marker::DELETEALL;
    pub_wall_list[i].publish(wall_list[0]);
  }
}
void Viewer::add_wall(int x, int y, int &idx, int dir) {

  Marker mk;
  double x1 = cell_size / 2 + cell_size * x;
  double y1 = cell_size / 2 + cell_size * y;

  double x2 = 0;
  double y2 = 0;
  double ang = PI / 2;
  if (dir == 0x04) {
    x2 = -cell_size / 2;
  } else if (dir == 0x08) {
    y2 = -cell_size / 2;
    ang = 0;
  } else if (dir == 0x01) {
    y2 = cell_size / 2;
    ang = 0;
  } else if (dir == 0x02) {
    x2 = cell_size / 2;
  }
  tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, ang);

  mk.id = idx;
  mk.ns = "map";
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.type = Marker::CUBE;
  mk.action = Marker::ADD;
  // mk.color.r = 1;
  // mk.color.g = 1;
  // mk.color.b = 1;
  // mk.color.a = 1;
  mk.color.r = 1;
  mk.color.g = 0;
  mk.color.b = 0;
  mk.color.a = 1;
  mk.pose.position.x = (x1 + x2) / 1000;
  mk.pose.position.y = (y1 + y2) / 1000;
  mk.pose.position.z = wall_height / 2 / 1000;
  mk.pose.orientation.x = quat[0];
  mk.pose.orientation.y = quat[1];
  mk.pose.orientation.z = quat[2];
  mk.pose.orientation.w = quat[3];
  mk.scale.x = (cell_size - poll_size) / 1000;
  mk.scale.y = poll_size / 1000;
  mk.scale.z = wall_height / 1000;

  wall_list[loop]->markers.emplace_back(mk);
  idx++;
}

void Viewer::add_poll(int x, int y, int &idx, int dir) {
  return;
  Marker mk;
  mk.id = idx;
  mk.ns = "map";
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.type = Marker::CUBE;
  mk.action = Marker::ADD;

  mk.color.r = 0.75;
  mk.color.g = 0.75;
  mk.color.b = 0.75;
  mk.color.a = 1;

  double x1 = cell_size / 2 + cell_size * x;
  double y1 = cell_size / 2 + cell_size * y;

  double x2 = 0;
  double y2 = 0;
  double ang = 0;
  if (dir == 1) {
    x2 = cell_size / 2;
    y2 = cell_size / 2;
  } else if (dir == 2) {
    x2 = -cell_size / 2;
    y2 = cell_size / 2;
  } else if (dir == 3) {
    x2 = cell_size / 2;
    y2 = -cell_size / 2;
  } else if (dir == 4) {
    x2 = -cell_size / 2;
    y2 = -cell_size / 2;
  }
  tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, ang);

  mk.pose.position.x = (x1 + x2) / 1000;
  mk.pose.position.y = (y1 + y2) / 1000;
  mk.pose.position.z = wall_height / 2 / 1000;
  mk.pose.orientation.x = quat[0];
  mk.pose.orientation.y = quat[1];
  mk.pose.orientation.z = quat[2];
  mk.pose.orientation.w = quat[3];
  mk.scale.x = poll_size / 1000;
  mk.scale.y = poll_size / 1000;
  mk.scale.z = wall_height / 1000;

  wall_list[loop]->markers.emplace_back(mk);
  idx++;
}
void Viewer::add_maze_base(int idx) {
  Marker mk;
  mk.id = idx;
  mk.ns = "maze_base";
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = Marker::CUBE;
  mk.action = Marker::ADD;
  // #263238
  mk.color.r = ((double)0x26 / 0xff);
  mk.color.g = ((double)0x32 / 0xff);
  mk.color.b = ((double)0x38 / 0xff);
  mk.color.a = 1;

  double x1 = cell_size * 8;
  double y1 = cell_size * 8;
  double x2 = 0;
  double y2 = 0;
  tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, 0);

  mk.pose.position.x = (x1 + x2) / 1000;
  mk.pose.position.y = (y1 + y2) / 1000;
  mk.pose.position.z = 0;
  mk.pose.orientation.x = quat[0];
  mk.pose.orientation.y = quat[1];
  mk.pose.orientation.z = quat[2];
  mk.pose.orientation.w = quat[3];
  mk.scale.x = cell_size * (maze_size + 2) / 1000;
  mk.scale.y = cell_size * (maze_size + 2) / 1000;
  mk.scale.z = 0.4;

  wall_list[loop]->markers.emplace_back(mk);
}
void Viewer::craete_wall() {

  wall_list[loop]->markers.resize(1);
  wall_list[loop]->markers[0].action = Marker::DELETEALL;

  int x = mz.ego.pos_x;
  int y = mz.ego.pos_y;
  int x_min = 0;
  int y_min = 0;
  int x_max = maze_size;
  int y_max = maze_size;
  int mode = 0;
  // if (x < (maze_size / 2) && y < (maze_size / 2)) {
  //   mode = 0;
  //   x_min = 0;
  //   y_min = 0;
  //   x_max = maze_size / 2;
  //   y_max = maze_size / 2;
  // } else if (x < (maze_size / 2) && y >= (maze_size / 2)) {
  //   mode = 1;
  //   x_min = 0;
  //   y_min = maze_size / 2;
  //   x_max = maze_size / 2;
  //   y_max = maze_size;
  // } else if (x >= (maze_size / 2) && y < (maze_size / 2)) {
  //   mode = 2;
  //   x_min = maze_size / 2;
  //   y_min = 0;
  //   x_max = maze_size;
  //   y_max = maze_size / 2;
  // } else if (x >= (maze_size / 2) && y >= (maze_size / 2)) {
  //   mode = 3;
  //   x_min = maze_size / 2;
  //   y_min = maze_size / 2;
  //   x_max = maze_size;
  //   y_max = maze_size;
  // }

  int c = 2;
  for (int i = x_min; i < x_max; i++) {
    for (int j = y_min; j < y_max; j++) {
      if (i == 0) {
        if ((map[i][j] & 0x04) == 0x04) {
          add_wall(i, j, c, 0x04);
        }
      }
      if (j == 0) {
        if ((map[i][j] & 0x08) == 0x08) {
          add_wall(i, j, c, 0x08);
        }
      }
      if ((map[i][j] & 0x01) == 0x01) {
        add_wall(i, j, c, 0x01);
      }
      if ((map[i][j] & 0x02) == 0x02) {
        add_wall(i, j, c, 0x02);
      }
      if (i == 0 && j == 0) {
        add_poll(i, j, c, 4);
      }
      if (i == (maze_size - 1) && j == (maze_size - 1)) {
        add_poll(i, j, c, 1);
      }
      add_poll(i, j, c, 2);
      add_poll(i, j, c, 3);
    }
  }
  pub_wall_list[mode].publish(wall_list[loop]);
}

void Viewer::processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  try {
    if (feedback->event_type ==
        visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK) {
      int x = feedback->mouse_point.x * 1000 / cell_size;
      int y = feedback->mouse_point.y * 1000 / cell_size;

      OverlayText text;
      text.action = jsk_rviz_plugins::OverlayText::ADD;
      text.width = 400;
      text.height = 200;
      text.left = 0;
      text.top = 0;

      std_msgs::ColorRGBA color1, color2;
      color1.r = 0;
      color1.g = 0;
      color1.b = 0;
      color1.a = 0.4;
      text.bg_color = color1;

      color2.r = 25.0 / 255;
      color2.g = 255.0 / 255;
      color2.b = 240.0 / 255;
      color2.a = 0.8;
      text.fg_color = color2;

      text.line_width = 1;
      text.text_size = 14;
      text.font = "Ubuntu";
      if (x < 0 || y < 0)
        return;
      if (x >= maze_size || y >= maze_size)
        return;
      char n = (map[x][y] & 0x01) != 0;
      char e = (map[x][y] & 0x02) != 0;
      char w = (map[x][y] & 0x04) != 0;
      char s = (map[x][y] & 0x08) != 0;
      char n2 = (map[x][y] & 0x10) != 0;
      char e2 = (map[x][y] & 0x20) != 0;
      char w2 = (map[x][y] & 0x40) != 0;
      char s2 = (map[x][y] & 0x80) != 0;
      snprintf(
          buf, TXT_BUF,
          "(x, y): (%2d, %2d)\n wall: (%d,%d,%d,%d)\n step: (%d,%d,%d,%d)\n", x,
          y, n, e, w, s, n2, e2, w2, s2);
      text.text = &buf[0];
      pub_text1.publish(text);
    }

  } catch (exception &e) {
  }
}

void Viewer::make_base(Marker &marker) {
  marker.type = Marker::CUBE;
  marker.scale.x = cell_size * (maze_size + 2) / 1000;
  marker.scale.y = cell_size * (maze_size + 2) / 1000;
  marker.scale.z = 0.2;
  marker.color.r = ((double)0x26 / 0xff);
  marker.color.g = ((double)0x32 / 0xff);
  marker.color.b = ((double)0x38 / 0xff);
  marker.color.a = 0.75;
}

void Viewer::makeButtonMarker(const tf::Vector3 &position) {

  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.name = "button";
  int_marker.description = "";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.orientation_mode = 1;
  control.name = "button_control";

  Marker marker;
  make_base(marker);
  control.markers.emplace_back(marker);
  control.always_visible = true;
  int_marker.controls.emplace_back(control);

  im_server->insert(int_marker);
  im_server->setCallback(int_marker.name,
                         boost::bind(&Viewer::processFeedback, this, _1));
  sub_maze = _nh.subscribe("/maze", 10, &Viewer::maze_callback, this);
  sub_path = _nh.subscribe("/path_info", 10, &Viewer::path_callback, this);
}

void Viewer::maze_callback(const my_msg::mazeConstPtr &_mz) {
  mz = *_mz;
  initflag = true;
  set_data();
}

void Viewer::path_callback(const my_msg::pathConstPtr &_path) {
  ph = *_path;
  initflag2 = true;
}
void Viewer::set_data() { //
  if (!initflag)
    return;
  maze_size = mz.maze_size;
  int c = 0;
  try {
    ego.x = mz.ego.pos_x;
    ego.y = mz.ego.pos_y;
    ego_dir = mz.ego.dir;
    for (int i = 0; i < maze_size; i++) {
      for (int j = 0; j < maze_size; j++) {
        map[i][j] = mz.wall[c];
        dist[i][j] = mz.dist[c];
        c++;
      }
    }
  } catch (exception &e) {
    ROS_WARN("index = %d", c);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "viewer");
  ros::NodeHandle nh;

  Viewer viewer;
  viewer.set_node_handler(nh);
  viewer.init();

  ros::spin();

  viewer.im_server.reset();
  return 0;
}

void Viewer::set_map() {}