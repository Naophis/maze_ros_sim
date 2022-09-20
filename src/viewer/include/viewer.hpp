#include "my_msg/maze.h"
#include "my_msg/path.h"
#include "stdio.h"
#include "tf/transform_listener.h"
#include <interactive_markers/interactive_marker_server.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "my_msg/candidate_route_map.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std;
using namespace visualization_msgs;
using namespace interactive_markers;
using namespace jsk_rviz_plugins;

constexpr char MAX_LOOP = 5;
constexpr char MAX_MAZE_SIZE = 32;
constexpr int TXT_BUF = 256;
constexpr double PI = 3.141592653589793238;

typedef struct {
  int x;
  int y;
} point_t;

class Viewer {
public:
  Viewer(/* args */);
  ~Viewer();
  void timer_callback(const ros::TimerEvent &e);
  void init();
  void set_node_handler(const ros::NodeHandle &nh) { _nh = nh; }
  void craete_wall();
  void add_wall(int x, int y, int &idx, int dir);
  void add_poll(int x, int y, int &idx, int dir);
  void add_maze_base(int idx);
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server;
  void processFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void makeButtonMarker(const tf::Vector3 &position);
  void make_base(Marker &marker);
  void set_map();
  void create_maze_info();
  void create_maze_info2(double x, double y, int &id, int dir);
  void create_maze_diainfo();

  void maze_callback(const my_msg::mazeConstPtr &_mz);
  void path_callback(const my_msg::pathConstPtr &_path);
  void candimap_callback(const my_msg::candidate_route_mapConstPtr &_path);

  void set_data();

  void create_ego_marker();
  void create_ego_marker2();

private:
  ros::Timer timer;
  ros::NodeHandle _nh;
  MarkerArrayPtr wall_list[MAX_LOOP];
  MarkerArrayPtr maze_info_list[MAX_LOOP];
  MarkerArrayPtr maze_diainfo_list[MAX_LOOP];
  MarkerArrayPtr ego_marker_list[MAX_LOOP];
  MarkerArrayPtr candidate_map_list[MAX_LOOP];
  tf2_ros::TransformBroadcaster dynamic_br_;

  MarkerPtr path_array_list[MAX_LOOP];
  ros::Publisher pub_wall_list[4];
  char map[MAX_MAZE_SIZE][MAX_MAZE_SIZE];
  int dist[MAX_MAZE_SIZE][MAX_MAZE_SIZE];
  int loop = 0;
  int maze_size = 16;
  double cell_size = 180;
  double poll_size = 12;
  double wall_height = 48;
  ros::Publisher pub_text1;
  ros::Publisher pub_maze_info;
  ros::Publisher pub_maze_diainfo;
  ros::Publisher pub_ego_marker;
  ros::Publisher pub_path_marker;
  ros::Publisher pub_candi_map;
  char buf[TXT_BUF];
  ros::Subscriber sub_maze;
  ros::Subscriber sub_path;
  ros::Subscriber sub_candidate_map;

  my_msg::maze mz;
  my_msg::path ph;
  my_msg::candidate_route_map candi_map;

  bool initflag = false;
  bool initflag2 = false;
  bool initflag3 = false;
  point_t ego;
  int ego_dir;
};

Viewer::Viewer(/* args */) {}

Viewer::~Viewer() {}
