
# Setup Instructions:

1. Install [catkin](http://wiki.ros.org/catkin) for ROS melodic.

2. Create a catkin [workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

3. Copy the `unity_communication` package to the `src` directory of your catkin workspace.

4. Build the catkin workspace:

```
$ cd ~/catkin_ws
$ catkin_make
```

5. Source the workspace's setup.sh file after calling `catkin_make` before trying to use the applications:

```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
```

6. Open a seperate terminal and launch roscore
```
$ roscore
```

7. Create rosbridge server in a separate terminal
```
$ roslaunch rosbridge_server rosbridge_websocket.launch
```

8. Run the executable ROS script
```
$ rosrun unity_communication <filename>
```

