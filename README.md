## Piggyback overall description

The piggyback is a system that assists the data logging in the context of the ERL competition benchmarks.
The system relies on an auxiliary computer running ROS on which the data is logged. The system also contains a battery to power the computer.
The computer is connected to the robot via ethernet.
 

## Piggyback ROS node

The piggyback ROS node must be run by executing `roslaunch transparent_benchmarking piggyback.launch team_name:=test_team benchmark_name:=test_benchmark`.
The piggyback ROS node subscribes and republishes the topics listed in the configuration file `transparent_benchmarking/config/{team-name}.yaml`.
The topics are republished with the ERL standard name and the messages received on each topic are counted.
A string containing the count of the number of messages received for each republished topic is publised on `/transparent_benchmarking/messages_count` everytime a message is received.
Also an overall count of all messages received is published on the topic `/transparent_benchmarking/overall_messages_count`.
The ROS bag is recorded by the rosbag record ROS node, also run from the launch file. The bags are saved on a USB pendrive.
The recording node actually records all topics, including the original topics and the topics not specified in the configuration file. Moreover, some metadata is recorded in the bag, such as team name and benchmark name.


# Configuration

Each team must provide a configuration file in which is specified the name of each topic relevant to the benchmarks. These are the only topics that are republished with their standard name.
These are all the topics required by the competition rulebook in section Internal Data Logging, plus some useful topics.
The following topics can be inserted in the yaml configuration file:
```
map_topic
map_metadata_topic
robot_pose_topic
robot_pose_with_covariance_topic
trajectory_topic
laser_scan_topics (list)
camera_image_topics (list)
camera_info_topics (list)
depth_sensor_topics (list)
audio_topics (list)
```

The configuration file *must* contain the team name.

For example, a minimal configuration file for the team "test_team" could be:
```
team_name: "test_team"
map_topic: "/map"
map_metadata_topic: "/map_metadata"
robot_pose_topic: "/my_localization/robot_pose"
```

In this case, all topics will be recorded and the topics specified in the configuration will also be republished and recorded. Specifically, the piggyback ROS node will publish the topics:
```
/ERL/map
/ERL/map_metadata
/ERL/robot_pose
/transparent_benchmarking/benchmark
/transparent_benchmarking/team_name
/transparent_benchmarking/messages_count
/transparent_benchmarking/overall_messages_count
```

## Piggyback and robot network configuration

The piggyback is connected to the robot through an ethernet cable and the connection is configured manually as a point to point connection.
The ROS nodes on the robot need to know the piggyback's nodes URI (i.e., hostname:port or IP:port) and vice versa, so some configuration must be done to let the systems know each other.


# Robot configuration

The connection is a simple manual connection: `IP=10.2.0.1`, `netmask=255.0.0.0`, `gateway=0.0.0.0`.
If NetworkManager is available, the connection can be set up from NetworkManager as follows (on Ubuntu 14.04 and later, using Unity): click on the NetworkManager applet icon; click on Edit Connections; click on Add; select Ethernet and click Create; in the field Ethernet / Device MAC address, select the ethernet interface to which the piggyback will be connected; in the field IPv4 Settings / Method, select Manual; in the table IPv4 Settings / Adresses, add a line with Address: 10.2.0.1, Netmask: 255.0.0.0, Gateway: 0.0.0.0; Finally, in the field Connection name, insert the desired connection name and click on Save.

To provide the URI of the piggyback to the nodes on the robot it is necessary to add the piggyback's hostname in the file /etc/hosts of the robot.
The following line is added to /etc/hosts, assuming the hostname of the piggyback is 'AIRBerry':
```
10.2.0.2	AIRBerry
```
This only needs to be done once on all robots since the piggyback hostname is never changed.


# Piggyback configuration

The connection is a simple manual connection similar to the one on the robot: `IP=10.2.0.2`, `netmask=255.0.0.0`, `gateway=0.0.0.0`.

Adding the hostname of the robot in the file /etc/hosts of the piggyback is necessary to provide the hostname of the robot to the piggyback nodes and allow them to subscribe to the topics advertised by nodes on the robot.
The correct robot hostnames can be found out by executing the command `hostname` on each robot.
Note that the hostname set in /etc/hosts must correspond to the IP address of the robot. For example the following line is added to /etc/hosts when attaching the piggyback to a robot with hostname 'Robot1':
```
10.2.0.1	Robot1
```
The robot hostname must be updated with the correct value from the robot before each benchmark. The IP address of the robot is supposed to stay the same for all robots.

On the piggyback, the environmental variable `ROS_MASTER_URI` is set with the command `export ROS_MASTER_URI=http://10.2.0.1:11311` from ~/.bashrc.
This tells the ROS environment on the piggyback that the ROS master is running on the robot with the standard port.
