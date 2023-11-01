FROM arm64v8/ros:noetic-robot

ENV USE_HOST_NETWORK=true

RUN sudo apt-get update
RUN sudo apt-get install -y git-all
RUN sudo apt-get install ros-noetic-naoqi-libqi
RUN sudo apt-get install ros-noetic-naoqi-libqicore
RUN sudo apt-get install ros-noetic-naoqi-bridge-msgs
RUN sudo apt install net-tools


RUN mkdir -p ~/catkin_ws/src
RUN cd ~/catkin_ws/src
RUN pwd
RUN git clone https://github.com/ros-naoqi/naoqi_driver.git ~/catkin_ws/src/naoqi_driver

RUN rosdep install -i -y --from-paths ~/catkin_ws/src/naoqi_driver
RUN . /opt/ros/noetic/setup.sh




