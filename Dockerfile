From ros:melodic-ros-base
RUN apt update && apt upgrade -y
RUN apt install vim git g++ build-essential make qt5-default qtcreator libarmadillo-dev

RUN echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc
RUN mkdir -p ~/catkin_ws/src
RUN cd ~/catkin_ws/src && catkin_init_workspace
RUN git clone https://github.com/OkDoky/prediction_layer.git
RUN git clone https://github.com/OkDoky/obstacle_detector.git -b melodic-devel
RUN apt install ros-melodic-navigation ros-melodic-turtlebot3*

RUN echo 'alias cs="cd ~/catkin_ws/src"' >> ~/.bashrc
RUN echo 'alias cw="cd ~/catkin_ws"' >> ~/.bashrc
RUN echo 'alias cm="cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=release"' >> ~/.bashrc

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENTRYPOINT [ "/ros_entrypoint.sh" ]
        CMD [ "bash" ]
