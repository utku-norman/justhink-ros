# ROS2 Wrappers for JUSThink Human-Robot Pedagogical Scenario

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview

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

* [Robot Operating System (ROS)](https://docs.ros.org) (middleware for robotics)
* [justhink_world](https://github.com/utku-norman/justhink_world) to represent an activity as a world/problem with a state (that depends on [pomdp_py](https://h2r.github.io/pomdp-py/html/), [networkx](https://networkx.org/), [pyglet](https://pyglet.readthedocs.io/en/latest/), [importlib_resources](https://importlib-resources.readthedocs.io/en/latest/), and [pqdict](https://pypi.org/project/pqdict/))

#### Building

1) [Create or reuse a ROS workspace](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) (e.g. `~/ros_ws`):
```
# Load the default workspace.
source /opt/ros/noetic/setup.bash

# Create and overlay your catkin workspace.
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
source devel/setup.bash
```

2) Clone these [justhink-ros] ROS packages folder inside the 'src' folder of your workspace:
```
cd ~/catkin_ws/src

git clone https://github.com/utku-norman/justhink-ros.git
```

3) For the dependencies, create a new Python [virtual environment](https://docs.python.org/3/tutorial/venv.html) and activate it. 
Do so in the same folder. 
Note that the folder name `venv` is [git-ignored](https://git-scm.com/docs/gitignore)):
```
cd ~/catkin_ws/src/justhink-ros

python3 -m venv .venv --prompt JUSThink-ROS-env

source .venv/bin/activate
```

If you do not have `venv`, first install it by: `sudo apt install python3-venv`

4) Install the dependency [justhink_world] Python package inside this virtual environment:
```
# Get the source code.
git clone https://github.com/utku-norman/justhink_world.git .venv/justhink_world

# Activate the virtual environment.
source .venv/bin/activate

# Install the package.
pip install -e .venv/justhink_world
```

For issues, details on installation and usage, refer to the [README](https://github.com/utku-norman/justhink_world/#readme) of [justhink_world].

5) Install Python dependencies for running ROS in the virtual environment.
```
pip install pyyaml rospkg empy
```

6) Install the ROS packages (while the virtual environment is activated): 
```
cd ~/catkin_ws

catkin build justhink_msgs

catkin build justhink_scenario

catkin build justhink_agent

catkin build qt_robot_interface qt_motors_controller qt_gesture_controller
catkin build justhink_robot

source devel/setup.bash
```

If you encounter error ``catkin not found'', install it by:
```
sudo apt-get install python3-catkin-tools
```


## Usage

0) Convenience, to add these to `.bash_aliases`:
```
alias sshqtrp="ssh developer@192.168.4.1"
alias sshqtpc="ssh qtrobot@192.168.100.2"

alias sourceros="source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash"

alias sourcejusthink="sourceros; source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; source ~/catkin_ws/src/justhink-ros/.venv/bin/activate"

alias sourceqtrobot="sourceros; export ROS_IP=192.168.4.114; export ROS_MASTER_URI=http://192.168.4.1:11311"
```

1) Set the volume level of the robot.
```
rosservice call /qt_robot/setting/setVolume 52    # for development
rosservice call /qt_robot/setting/setVolume 80    # for experiment

# Test robot speech, gesture and facial expression topics.
rostopic pub -1 /qt_robot/speech/say std_msgs/String "data: 'Hi'"
rostopic pub -1 /qt_robot/emotion/show std_msgs/String "data: 'QT/happy'"
rostopic pub -1 /qt_robot/emotion/show std_msgs/String "data: 'QT/kiss'"
rostopic pub -1 /qt_robot/gesture/play std_msgs/String "data: 'QT/happy'"
```

2) Start the ROS logger in a terminal:
```
sourcejusthink
sourceqtrobot

NO=15 # Student No
rosrun justhink_robot run_justhink_recorder.sh $NO
```


3) start screen recording in another terminal:
```
sourcejusthink

NO=15 # Student No
rosrun justhink_scenario run_screen_recorder.sh $NO
```


4) start robot side logging in another terminal:
```
sshqtpc

NO=15 # Student No

cd justhink/
./run_qtrobot_recorder.sh
```


5) Start the robot node in another terminal:
```
sourcejusthink
sourceqtrobot

export ROS_LOG_DIR=$(rospack find justhink_robot)/data/log
# rm $ROS_LOG_DIR/agent_embodiment.log

export ROS_NAMESPACE=agent
rosrun justhink_robot run_robot.py

# Test
rostopic pub -1 /agent/embodiment/say std_msgs/String "data: 'Hi'"
rosservice call /qt_robot/motors/home "['HeadPitch', 'HeadYaw', 'left_arm', 'right_arm']"  # Home all
```


6) Start the agent node in a third terminal:
```
sourcejusthink
sourceqtrobot

export ROS_LOG_DIR=$(rospack find justhink_agent)/data/log
# rm $ROS_LOG_DIR/agent_cognition.log

export ROS_NAMESPACE=agent
rosrun justhink_agent run_agent.py _mode:=optimal

rosrun justhink_agent run_agent.py _mode:=greedy

rosrun justhink_agent run_agent.py _mode:=aligning




rosrun justhink_agent run_agent.py _instruct:=False _mode:=optimal
rosrun justhink_agent run_agent.py _instruct:=False _mode:=greedy
rosrun justhink_agent run_agent.py _instruct:=False _mode:=aligning


rosrun justhink_agent run_agent.py _instruct:=True

```


7) Start the situation node (i.e. the learning scenario) in a fourth terminal:
```
sourcejusthink
sourceqtrobot

export ROS_LOG_DIR=$(rospack find justhink_scenario)/data/log
# rm $ROS_LOG_DIR/env_situation.log


export ROS_NAMESPACE=env
rosrun justhink_scenario run_scenario.py


rosrun justhink_scenario run_scenario.py _entry:=collaboration-1
rosrun justhink_scenario run_scenario.py _entry:=collaboration-2


rosrun justhink_scenario run_scenario.py _robot_text:=True

```



Backup
```
rsync -avuzh --progress qtrobot@192.168.100.2:/home/qtrobot/justhink ~/data

sshqtpc

```


### Running with a touch screen

#### Mapping the touch interface onto the touch screen

Check the name of the touch controller, e.g. "USBest Technology SiS HID Touch Controller"
```
xinput
```

2) Check the name of the screen, e.g. "DP-3"
```
xrandr -q
```

3) Map the touch controller to the screen, e.g., if it is DP-3 from the previous step:
```
xinput map-to-output "USBest Technology SiS HID Touch Controller" DP-3
```

#### Hiding the cursor on touch events

Install the fork of unclutter that hides the cursor for touch only (The default unclutter from apt does not have "-touch".)
```
sudo apt install asciidoc libev-dev libxslt1-dev docbook-xsl xsltproc libxml2-utils    # Prerequisites
git clone https://github.com/nowrep/unclutter-xfixes.git
cd unclutter-xfixes
make
sudo make install
```

5) Run unclutter on a separate terminal. Touch on the screen will not show cursor.
```
unclutter -touch
```

#### Rotating the screen by 180 degrees
To prevent the power button being pressed accidentally (normally bottom right corner, if rotated top left corner)

1) In Display setting of Ubuntu, change Rotation to 180 degrees.

2) Remap the touch upside-down.
```
xinput set-prop "USBest Technology SiS HID Touch Controller" --type=float "Coordinate Transformation Matrix" 0 -1 1 1 0 0 0 0 1
```







## Acknowledgements

This project has received funding from the European Union's Horizon 2020 research and innovation programme under grant agreement No 765955. Namely, the [ANIMATAS Project](https://www.animatas.eu/).

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/utku-norman/justhink-ros/issues).


[ROS]: http://www.ros.org
[justhink_world]: https://github.com/utku-norman/justhink_world
[justhink-ros]: https://github.com/utku-norman/justhink-ros