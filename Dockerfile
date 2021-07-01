FROM  ghcr.io/europeanroverchallenge/erc-remote-image-base:latest

RUN sudo apt update

RUN sudo apt -y install ros-melodic-rospy-tutorials vim
RUN python3 -m pip install --upgrade pip 
RUN pip3 install opencv-python matplotlib rospkg pynput

WORKDIR /motion_control
COPY src ./src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /motion_control; catkin build'
