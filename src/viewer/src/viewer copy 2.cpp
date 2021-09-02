#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <math.h>

using namespace visualization_msgs;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

// %Tag(Box)%
Marker makeBox(InteractiveMarker &msg) {
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl &makeBoxControl(InteractiveMarker &msg) {
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.emplace_back(makeBox(msg));
  msg.controls.emplace_back(control);

  return msg.controls.back();
}

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
    << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if (feedback->mouse_point_valid) {
    mouse_point_ss << " at " << feedback->mouse_point.x << ", "
                   << feedback->mouse_point.y << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch (feedback->event_type) {
  case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    ROS_INFO_STREAM(s.str() << ": button click" << mouse_point_ss.str() << ".");
    ROS_WARN("click");
    break;

  case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
    ROS_INFO_STREAM(s.str() << ": menu item " << feedback->menu_entry_id
                            << " clicked" << mouse_point_ss.str() << ".");
    break;

  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    ROS_INFO_STREAM(s.str()
                    << ": pose changed"
                    << "\nposition = " << feedback->pose.position.x << ", "
                    << feedback->pose.position.y << ", "
                    << feedback->pose.position.z
                    << "\norientation = " << feedback->pose.orientation.w
                    << ", " << feedback->pose.orientation.x << ", "
                    << feedback->pose.orientation.y << ", "
                    << feedback->pose.orientation.z
                    << "\nframe: " << feedback->header.frame_id
                    << " time: " << feedback->header.stamp.sec << "sec, "
                    << feedback->header.stamp.nsec << " nsec");
    break;

  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
    ROS_INFO_STREAM(s.str() << ": mouse down" << mouse_point_ss.str() << ".");
    break;

  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
    ROS_INFO_STREAM(s.str() << ": mouse up" << mouse_point_ss.str() << ".");
    break;
  }

  server->applyChanges();
}

////////////////////////////////////////////////////////////////////////////////////

void makeButtonMarker(const tf::Vector3 &position) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "button";
  int_marker.description = "Button\n(Left Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  Marker marker = makeBox(int_marker);
  control.markers.emplace_back(marker);
  control.always_visible = true;
  int_marker.controls.emplace_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "basic_controls");
  ros::NodeHandle n;

  server.reset(new interactive_markers::InteractiveMarkerServer(
      "basic_controls", "", false));

  ros::Duration(0.1).sleep();
  tf::Vector3 position;
  position = tf::Vector3(0, 0, 0);
  makeButtonMarker(position);

  server->applyChanges();

  ros::spin();

  server.reset();
}