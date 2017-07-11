#!/bin/bash

########################
# add repos
########################
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    sudo add-apt-repository ppa:webupd8team/sublime-text-3
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

########################
# update
########################
    sudo apt-get update

########################
# install
########################
# install tree
    sudo apt-get install tree

# install ROS
    sudo apt-get install ros-kinetic-desktop-full
    sudo rosdep init
    rosdep update

    # install rosinstall
    sudo apt-get install python-rosinstall

# install Gazebo
    wget -O /tmp/gazebo5_install.sh http://osrf-distributions.s3.amazonaws.com/gazebo/gazebo5_install.sh; sudo sh /tmp/gazebo5_install.sh

# install Sublime 3
    sudo apt-get install sublime-text-installer

# catkin tools
    sudo apt-get install python-catkin-tools

# install mavros
    sudo apt-get install ros-kinetic-mavros

# install navigation stack
    sudo apt-get install ros-kinetic-navigation

########################
# setup environment
########################
# setup bashrc
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc

# clone ursa server
git --recursive clone https://github.com/ursa-drone/ursa-server.git
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y












