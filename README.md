# How to use this package

## Setup
Make sure to install Ubuntu 18.04 Bionic Beaver and ROS Melodic Morenia or any equivalent<br>
Install packages libqt4-dev ros-melodic-map-server<br>
Install stdr simulator package<br>
Clone this repository into the source folder<br>

## Using the package
### Without GUI
Use roslaunch teleoperator teleoperator.launch

### With GUI
Use roslaunch teleoperator teleoperator_with_stdr.launch

#### NOTE: controlling more than one robot at the same time
After the above commands, add namespace:=\<robot_name\><br>
Be sure not to name any robots the same name