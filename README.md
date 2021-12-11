# ROS2 Wrappers for JUSThink Human-Robot Pedagogical Scenario

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview

This repository contains the [ROS] package to run the pedagogical scenario that contains a human-robot collaborative learning activity for school children, named [JUSThink](https://www.epfl.ch/labs/chili/index-html/research/animatas/justhink/). The scenario aims to improve their computational thinking skills by applying abstract and algorithmic reasoning to solve an unfamiliar problem on networks.

* In an individual activity, a human learner is given a network of gold mines with possible positions for railway tracks, where each track if it is built connects one mine to another. The cost of each track is visible. The goal is to collect the gold by connecting the gold mines to each other, while spending as little as possible to build the tracks.
* In a collaborative activity, the human and the robot as (same-status) peers collaboratively construct a solution to this problem by deciding together which tracks to build, and submit it as their solution to the system. They take turns in suggesting to select a specific connection, where the other either agrees or disagrees with this suggestion. A track will be built only if it is suggested by one and accepted by the other.

**Keywords:** artificial intelligence, human-robot interaction, mutual understanding, collaborative learning, computational thinking


### License

The whole package is under MIT License, see [LICENSE](LICENSE).

This README is based on the project [ros_best_practices](https://github.com/leggedrobotics/ros_best_practices), Copyright 2015-2017, Péter Fankhauser. It is licensed under the BSD 3-Clause Clear License. See [doc/LICENSE](doc/LICENSE) for additional details.

**Author: Utku Norman<br />
Affiliation: [CHILI Lab, EPFL](https://www.epfl.ch/labs/chili/)<br />
Maintainer: Utku Norman, utku.norman@epfl.ch**

The packages in [justhink-ros] has been tested under [ROS2] Foxy on Ubuntu 20.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


## Installation

### Building from Source

#### Dependencies

* [Robot Operating System (ROS 2)](https://docs.ros.org) (middleware for robotics)
* [justhink_world](https://github.com/utku-norman/justhink_world) to represent an activity as a world/problem with a state (that depends on [pomdp_py](https://h2r.github.io/pomdp-py/html/), [networkx](https://networkx.org/), [pyglet](https://pyglet.readthedocs.io/en/latest/), [importlib_resources](https://importlib-resources.readthedocs.io/en/latest/), and [pqdict](https://pypi.org/project/pqdict/))

#### Building

1) [Create or reuse a ROS2 workspace](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) (e.g. `~/ros_ws`):
```
# Load the default workspace.
source /opt/ros/foxy/setup.bash

# Create and overlay your own workspace.
mkdir -p ~/ros_ws/src

cd ~/ros_ws/
```

2) Clone this [justhink-ros] ROS2 packages folder inside the 'src' folder of your workspace and switch to the `ros2` branch:
```
cd ~/ros_ws/src

git clone https://github.com/utku-norman/justhink-ros.git
git checkout ros2
```

3) For the dependencies, create a new Python [virtual environment](https://docs.python.org/3/tutorial/venv.html) and activate it. 
Do so in the same folder. 
Note that the folder name `venv` is [git-ignored](https://git-scm.com/docs/gitignore)):
```
cd ~/ros_ws/src/justhink-ros

python3 -m venv venv
touch venv/COLCON_IGNORE

source venv/bin/activate
```

If you do not have `venv`, first install it by: `sudo apt install python3-venv`

4) Install the dependency [justhink_world] Python package inside this virtual environment:
```
cd ~/ros_ws/src/justhink-ros/venv

# Get the source code.
git clone https://github.com/utku-norman/justhink_world.git

# Activate the virtual environment.
source bin/activate

# Go into the source code and install it.
cd justhink_world
pip install -e .

# Install Python dependencies for running ROS in the virtual environment.
# pip install empy catkin_pkg lark
```

For issues, details on installation and usage, refer to the [README](https://github.com/utku-norman/justhink_world/#readme) of [justhink_world].

5) Install these packages:
```
cd ~/ros_ws

source /opt/ros/foxy/setup.bash

colcon build --packages-up-to justhink_situation

. install/local_setup.bash
```


## Usage

```
cd ~/ros_ws

source /opt/ros/foxy/setup.bash
source install/local_setup.bash

export PYTHONPATH=$PYTHONPATH:/home/utku/ros_ws/src/justhink-ros/venv/lib/python3.8/site-packages/

ros2 run justhink_situation show_situation
```

## Nodes

### justhink_situation

Launches a simple application to monitor mouse and key events in ROS2.
Default size is 1920x1080, and the window is by default moved to the external screen.

A screenshot from the window:


<img src="doc/screenshot.png" width="480" />


The ROS computation graph (as visualised by [rqt_graph](http://wiki.ros.org/rqt_graph)) is as follows:


<img src="doc/rosgraph.png" width="480" />


#### Subscribed Topics

None.


#### Published Topics


* **`agent_intention`** ([[justhink_interfaces/Action]](https://github.com/utku-norman/justhink-ros/justhink_interfaces/blob/main/msg/Mouse.msg))

	Intended action of the agent/robot.
	Use `A` key on the activity to publish on this topic.
	Note that if you use `CTRL+A`, the action will also be executed directly.

	For example, one can monitor the action with

			ros2 topic echo /justhink_situation/agent_intention


	If you encounter the error: `ModuleNotFoundError: No module named 'justhink_interfaces'`, make sure you source the workspace by:

			cd ~/ros_ws
			source install/local_setup.bash


* **`mouse_motion`** ([[justhink_interfaces/Mouse]](https://github.com/utku-norman/justhink-ros/justhink_interfaces/blob/main/msg/Mouse.msg))

	Mouse movements that have position and button information, with a header that contains a timestamp and an activity name.

	For example, one can monitor the mouse motion events with

			ros2 topic echo /justhink_situation/mouse_motion


* **`mouse_press`** ([[justhink_interfaces/Mouse]](https://github.com/utku-norman/justhink-ros/justhink_interfaces/blob/main/msg/Mouse.msg))

	Mouse clicks that have position and button information, with a header that contains a timestamp and an activity name.

	For example, one can monitor the mouse press events with

			ros2 topic echo /justhink_situation/mouse_press

* **`mouse_drag`** ([[justhink_interfaces/Mouse]](https://github.com/utku-norman/justhink-ros/justhink_interfaces/blob/main/msg/Mouse.msg))

	Mouse drags that have position, position difference and mouse button information, with a header that contains a timestamp and an activity name.

	For example, one can monitor the mouse drag events with

			ros2 topic echo /justhink_situation/mouse_drag

* **`mouse_release`** ([[justhink_interfaces/Mouse]](https://github.com/utku-norman/justhink-ros/justhink_interfaces/blob/main/msg/Mouse.msg))

	Mouse releases that have position and button information, with a header that contains a timestamp and an activity name.

	For example, one can monitor the mouse release events with

			ros2 topic echo /justhink_situation/mouse_release

* **`key_press`** ([[justhink_interfaces/Key]](https://github.com/utku-norman/justhink-ros/justhink_interfaces/blob/main/msg/Key.msg))

	Key presses on the keyboard that have the symbol and modifiers information, with a header that contains a timestamp and an activity name.

	For example, one can monitor the key press events with

			ros2 topic echo /justhink_situation/key_press

* **`key_release`** ([[justhink_interfaces/Key]](https://github.com/utku-norman/justhink-ros/justhink_interfaces/blob/main/msg/Key.msg))

	Key releases on the keyboard that have the symbol and modifiers information, with a header that contains a timestamp and an activity name.

	For example, one can monitor the key release events with

			ros2 topic echo /justhink_situation/key_release


## Acknowledgements

This project has received funding from the European Union's Horizon 2020 research and innovation programme under grant agreement No 765955. Namely, the [ANIMATAS Project](https://www.animatas.eu/).

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/utku-norman/justhink_situation/issues).


[ROS2]: http://www.ros.org
[justhink_world]: https://github.com/utku-norman/justhink_world