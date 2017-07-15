#!/bin/bash
echo "########################"
echo "# add repos"
echo "########################"
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    echo "Y" | sudo add-apt-repository ppa:webupd8team/sublime-text-3
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -


echo "########################"
echo "# update"
echo "########################"
    sudo apt-get update

echo "########################"
echo "# install"
echo "########################"
# install tree
    sudo apt-get install tree

# install ROS
    echo "Y" | sudo apt-get install ros-kinetic-desktop-full
    # setup bashrc for future sessions (and this session)
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo rosdep init
    rosdep update
    # install rosinstall
    sudo apt-get install python-rosinstall

# install Sublime 3
    sudo apt-get install sublime-text-installer

# catkin tools
    echo "Y" | sudo apt-get install python-catkin-tools

# install mavros
    echo "Y" | sudo apt-get install ros-kinetic-mavros

# install mavros extras
    sudo apt-get install ros-kinetic-mavros-extras

# install navigation stack
    echo "Y" | sudo apt-get install ros-kinetic-navigation

# install iPython
    echo "Y" | sudo apt-get install ipython

# install protobuf compiler
    sudo apt-get install protobuf-compiler

# install python jinja
    sudo apt-get install python-jinja2

echo "########################"
echo "# setup environment"
echo "########################"
# setup bashrc
    echo "source ~/ursa-server/devel/setup.bash" >> ~/.bashrc
    echo "alias rtl=\"rostopic list\"" >> ~/.bashrc
    echo "alias rte=\"rostopic echo\"" >> ~/.bashrc
    echo "alias rpl=\"rosparam list\"" >> ~/.bashrc
    source ~/.bashrc

# clone ursa-server submodules and build
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
catkin build

# clone Firmware
cd $HOME
git clone --recursive https://github.com/ursa-drone/Firmware.git








