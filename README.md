This is sample catkin workspace with packages for ROS Indigo or above. Example package contains ros node with block definitions for Snap and can be run on target devices.
Installing ROS and rosbridge: http://wiki.ros.org/ROS/Installation http://wiki.ros.org/rosbridge_suite

Preparing example python process for run:
<pre>
cd <workspace_dir>
catkin_init_workspace
catkin_make
. devel/setup.bash
</pre> 

Remember to launch rosmaster and rosbridge:
<pre>
roscore
roslaunch rosbridge_server rosbridge_websocket.launch
roslaunch example example.launch
</pre>

Open modified Snap! website version (https://github.com/markol/Snap--ROS) and from project menu choose "Search for devices" option. ExapmpleBot should appear on the devices list.