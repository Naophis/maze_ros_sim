# maze_ros_sim

## dependency environment
* ROS(kinetic, melidic, noetic)

## how to use

### build
```shell
$ cd <this project>
$ catkin_make 
```

### launch rviz

```shell
$ cd <this project>
$ source devel/setup.sh
$ roslaunch viewer rviz.launch
```

### start sim

```shell
$ cd <this project>
$ source devel/setup.sh
roslaunch viewer visualizer.launch
```

## change sim maze

edit `src/viewer/launch/visualizer.launch`

```xml
....
<!-- remove comment out , you can simulate this scenario(maze pattern). -->
<rosparam command="load" file="$(find maze_solver)/config/halfsize/japan2010hef.yaml" /> 
<!-- <rosparam command="load" file="$(find maze_solver)/config/halfsize/japan2011hef.yaml" /> -->
...
```

## sim param 
edit `src/viewer/launch/visualizer.launch`

```xml
<!-- All maze pattern is given, so mouse does not search in maze. -->
<param name="maze_data/known" value="1" />  

<!-- maze pattern is not given, so mouse search in maze.  -->
<!-- <param name="maze_data/known" value="0" /> --> 
```