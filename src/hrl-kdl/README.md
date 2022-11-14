hrl-kdl
=======

Kinematics and geometry utilities for KDL

See documentation here: http://wiki.ros.org/hrl-kdl

This forks includes support for ROS noetic and python 3. Make sure to checkout to the 'noetic-devel' branch.

# Installing:

See below for installing:

- Clone the hrl_kdl repository in your catkin-ws folder
       
      cd ~/catkin_ws/src/
      git clone https://github.com/amir-yazdani/hrl-kdl.git
      
- Checkout to the noetic-devel branch
      
      cd hrl-kdl
      git checkout noetic-devel

- Install pykd_utils

      cd pykdl_utils
      python3 setup.py build
      sudo python3 setup.py install

- Install hrl_geom\

      cd ~/catkin_ws/src/hrl-kdl/hrl_geom/
      python3 setup.py build
      sudo python3 setup.py install

- Install urdf_parser and urdfdom-py

      sudo apt-get install ros-noetic-urdf-parser-plugin
      sudo apt-get install ros-noetic-urdfdom-py

- Build the catkin workspace

      cd ~/catkin_ws
      catkin build
