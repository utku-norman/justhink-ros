# JUSThink ROS Headers

## Overview

This repository contains the custom message and service headers for the JUSThink human-robot collaborative learning activity, to mediate the communication between its [ROS] packages. A human learner participates in the pedagogical scenario through an application ([justhink_scenario]). The robot behaviour is generated by [justhink_agent] and manifested (on e.g. [QTrobot]) by [justhink_robot].

### License

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

The whole package is under MIT License, see [LICENSE](LICENSE).

**Author: Utku Norman<br />
Affiliation: [CHILI Lab, EPFL](https://www.epfl.ch/labs/chili/)<br />
Maintainer: Utku Norman, utku.norman@epfl.ch**

The [justhink_msgs] package has been tested with [ROS Noetic](http://wiki.ros.org/noetic) and Python 3.8 (in Ubuntu 20.04 LTS).
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


## Installation

1) Clone this ([justhink_msgs]) ROS package inside the 'src' folder of the catkin workspace, e.g. `~/catkin_ws/src`.
```
cd ~/catkin_ws/src
git clone https://github.com/utku-norman/justhink_msgs.git
```

2) Build the ROS package.
```
cd ~/catkin_ws
catkin build justhink_msgs
```

3) Update the current shell environment:
```
source ~/catkin_ws/devel/setup.bash
```

4) Check the installation by trying to import the messages and services in a Python interpreter:
```
import justhink_msgs.msg
import justhink_msgs.srv
```

[ROS]: http://www.ros.org
[QTrobot]: https://luxai.com
[justhink_msgs]: https://github.com/utku-norman/justhink_msgs
[justhink_scenario]: https://github.com/utku-norman/justhink_scenario
[justhink_agent]: https://github.com/utku-norman/justhink_agent
[justhink_robot]: https://github.com/utku-norman/justhink_robot