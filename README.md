# RoboND-HomeServiceRobot
#### Update system and ensure that ROS kinetic full desktop is already installed.

```bash
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo apt-get install ros-kinetic-navigation
```



#### Create catkin workspace

```bash
mkdir -p ~/catkin_ws/src
```


#### Make your empty workspace to ensure that ROS installation is fine

```bash
cd ~/catkin_ws
catkin_make
```


#### Initialize git

```bash
cd ~/catkin_ws
git init
git commit -m "first commit"
git remote add origin https://github.com/<your newly created repository name>.git
git push -u origin master
```


#### Add required ROS packages as *git submodules*

```bash
cd ~/catkin_ws/src
git submodule add https://github.com/ros-perception/slam_gmapping.git
git submodule add https://github.com/turtlebot/turtlebot.git
git submodule add https://github.com/turtlebot/turtlebot_interactions.git
git submodule add https://github.com/turtlebot/turtlebot_simulator.git
git submodule add https://github.com/turtlebot/turtlebot_apps.git
```


#### Install ROS dependencies

```bash
sudo rosdep -i install slam_gmapping
sudo rosdep -i install turtlebot
sudo rosdep -i install turtlebot_teleop
sudo rosdep -i install turtlebot_rviz_launchers
sudo rosdep -i install turtlebot_apps
```


#### Build catkin workspace

```bash
$ cd ~/catkin_ws
$ catkin_make
```


#### source the development folder

```bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```


#### Catkin Workspace Structure

Should look something like this:
```
catkin_ws/src
    |-- slam_gmapping
        |-- gmapping
        |-- ...
    |-- turtlebot
        |-- turtlebot_teleop
        |-- ...
    |-- turtlebot_interactions
        |-- turtlebot_rviz_launchers
        |-- ...
    |-- turtlebot_simulator
        |-- turtlebot_gazebo
        |-- ...
    |-- turtlebot_apps
        |-- turtlebot_navigator
        |-- ...
    |-- add_markers
        |-- src/add_markers_node.cpp
        |-- ...
    |-- pick_objects
        |-- src/pick_objects_node.cpp
        |-- ...
    |-- RvizConfig
        |-- ...
    |-- ShellScripts
        |-- ...
    |-- Worlds
        |-- ...
    |-- wall_follower
        |-- src/add_markers_node.cpp
        |-- ...
```