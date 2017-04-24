
# Piggyback ROS node

The piggyback ROS node must be run by executing `roslaunch transparent_benchmarking piggyback.launch team_name:=test_team`.
The piggyback ROS node subscribes and republishes the topics listed in the configuration file `transparent_benchmarking/config/{team-name}.yaml`.
The topics are republished with the ERL standard name and the messages received on each topic are counted.
This count is publised on the topic `/transparent_benchmarking/messages_count` everytime a message is received.
The ROS bag is recorded by the rosbag record ROS node, also run from the launch file.


# Piggyback and robot network configuration

The piggyback is connected to the robot through an ethernet cable and the connection is configured manually as a point to point connection.

The ROS nodes on the robot need to know the piggyback's nodes URI (i.e., hostname:port or IP:port) and vice versa, so some configuration must be done to let the systems know each other:

# Robot configuration
The connection is a simple manual connection: `IP=10.2.0.1`, `netmask=255.0.0.0`, `gateway=0.0.0.0`.

To provide the URI of the piggyback to the nodes on the robot it is necessary to add the piggyback's hostname in the file /etc/hosts of the robot.
The following line is added to /etc/hosts, assuming the hostname of the piggyback is 'AIRBerry':
```
10.2.0.2	AIRBerry
```

# Piggyback configuration
The connection is a simple manual connection similar to the one on the robot: `IP=10.2.0.2`, `netmask=255.0.0.0`, `gateway=0.0.0.0`.

Adding the hostname of the robot in the file /etc/hosts of the piggyback is necessary to provide the hostname of the robot to the piggyback nodes and allow them to subscribe to the topics advertised by nodes on the robot.
The correct robot hostnames can be found out by executing the command `hostname` on each robot.
Note that the hostname set in /etc/hosts must correspond to the IP address of the robot. For example the following line is added to /etc/hosts when attaching the piggyback to a robot with hostname 'Robot1':
```
10.2.0.1	Robot1
```

On the piggyback, the environmental variable `ROS_MASTER_URI` is set with the command `export ROS_MASTER_URI=http://10.2.0.1:11311` from ~/.bashrc.
This tells the ROS environment on the piggyback that the ROS master is running on the robot with the standard port.
