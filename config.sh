#!/bin/bash

ROS_IMAGE="osrf/ros:humble-desktop-full"
VNC_IMAGE="dorowu/ubuntu-desktop-lxde-vnc:focal"
ARM_IMAGE="dorowu/ubuntu-desktop-lxde-vnc:focal-arm64"
WS_ROS="waver_ws"

DOCKER_IMAGE_NAME="waver-humble-image"
CONTAINER_NAME="waver-humble-container"
ROS_NETWORK="host"
