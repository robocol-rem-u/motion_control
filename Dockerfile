# build on top of the erc provided base image
FROM  ghcr.io/europeanroverchallenge/erc-remote-image-base:latest
# install some dependencies. Vim for quick viewing of scripts inside the cli.
RUN apt update && apt -y upgrade && apt install -y \
    python3-vcstool \
    ros-melodic-rospy-tutorials \
    vim
     
# install python requirements 
RUN python3 -m pip install --upgrade pip 
RUN pip3 install \
    opencv-python \
    matplotlib \
    rospkg \
    pynput

# copy .repos file in for fast cloning of remote repository if necessary
COPY motion.repos /

# build ROS workspace
WORKDIR /motion_control
COPY src ./src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /motion_control; catkin build'
