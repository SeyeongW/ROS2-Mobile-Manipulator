ARG IMAGE="osrf/ros:humble-desktop-full"

FROM ${IMAGE}

ARG OS
ARG WS_ROS
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV USER=root
ENV WS=/${WS_ROS}
WORKDIR ${WS}

RUN apt update && apt install -y \
    python3-pip \
    python3-opencv \
    git \
    nano \
    iputils-ping \
    net-tools

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-turtlebot3*

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-robot-localization 

COPY ./autostart.sh /${WS_ROS}
RUN chmod +x /${WS_ROS}/autostart.sh
RUN ./autostart.sh
