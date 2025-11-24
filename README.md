# Node Manager

The goal of node manager is to allow managing ROS2 nodes on a remote machine using messages send via ROS2. This approach allows dynamic reconfiguration and setup of nodes, without requiring stuff like ssh connection.

## Dependencies
This package was developed on ROS2 Jazzy. It also depends on 
boost cpp libraries.

## Installation

This repository is a ROS2 package, so installation is simple.
Clone it into your workspace and build with colcon. Then run it using provided launch file.

```
cd ~/ros2_ws/scr
git clone https://github.com/xonder05/node_manager.git
cd ..
colcon build
source ~/ros2_ws/install/setup.bash
ros2 launch node_manager node_manager_launch.py
```

## Communication protocol

Commands to the node are to be sent using `management/commands` topic. Responses will be sent via the same topic.
(this part is subject to changes since using service is obviously better but due to compatibility i had to use topics)

### Message fields:

| Field        | Type  |
| ------------ | ----- |
| message_type | uint8 |

There are currently four following message types
- 10 - run
- 20 - launch
- 50 - stop
- 90 - response

| Field        | Type   |
| ------------ | ------ |
| manager_id   | string |

Used to identify specific manager in case single controller 
commands multiple managers that share same topic. Is set as 
node parameter.

| Field        | Type   |
| ------------ | ------ |
| node_id      | string |

Should uniquely identify running nodes. Used for stopping.
Trying to run node with same id will kill the original one.

| Field        | Type  |
| ------------ | ----- |
| return_value | uint8 |
 
Values returned from manager as part of response message.

- 00 - success
    - 01 - node started successfully
    - 04 - parameter change successfully
    - 05 - node stopped successfully

- 10 - warnings
    - 11 node with same configuration is already running
    - 15 node cannot be stopped because it does not exist

- 20 - errors
    - 25 - process could not be stopped even with sigkill
    - 26 - waitpid error

| Field        | Type   |
| ------------ | ------ |
| package_name | string |

Used to specify which node to run or launch file to call.

| Field        | Type   |
| ------------ | ------ |
| node_name    | string |

Used to specify which node to run or launch file to call. Yes in case or sending launch message then this is used for specifying launch file name.


| Field        | Type   |
| ------------ | ------ |
| param_json   | string |

Used to give parameters or arguments to the node or launch file. It should be single json object with key-value pairs, where key is parameter name and value is parameter value. 
Eg: `{"use_sim_time":"true","robot_name":"waferbot"}`

| Field        | Type   |
| ------------ | ------ |
| remap_json   | string |

For passing remaps to nodes. This does not work for launch files. The format is same as for parameters.
