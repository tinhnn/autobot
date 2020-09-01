# autobot_BSP_pi
Common ROS packages for the Autobot, useable for both simulation and real robot operation.

## Install ROS Melodic
Reference [this link](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi)


#### Note: Building the catkin Workspace with option "-j1" because Pi3 only have 1G RAM
```
$ sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j1
```

## Install pigpio
```
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
```
Auto run pigpiod at startup
```
sudo systemctl enable pigpiod
```
## Create catkin workspace
Create a ROS Catkin workspace to contain our ROS packages:
```
# create the catkin workspace
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin_make

# add catkin_ws path to bashrc
$ sudo sh -c 'echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc'
```
Close and open a new terminal window. Verify that your catkin_ws is visible to ROS:
```
$ echo $ROS_PACKAGE_PATH 
/home/pi/catkin_ws/src:/opt/ros/melodic/share
```

## Build autobot

clone the repo
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/tinhnn/autobot_BSP_pi

# build the package
$ cd ../    # cd ~/catkin_ws
$ catkin_make

# confirm that autobot package can be found
$ rospack list
...
autobot_base /home/pi/catkin_ws/src/autobot_base
autobot_bringup /home/pi/catkin_ws/src/autobot_bringup
autobot_msgs /home/pi/catkin_ws/src/autobot_msgs
...
```

## Run autobot
* Terminal 1
```
$ roscore
```
* Terminal 2
```
$ roslaunch autobot_bringup autobot_robot.launch
```
