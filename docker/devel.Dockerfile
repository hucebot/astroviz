FROM ros:rolling
ENV ROS_DISTRO rolling

ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=0

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="Europe/Paris"

LABEL maintainer="clemente.donosok@gmail.com"

RUN apt-get update && apt-get upgrade -y

###### Install Python Dependencies
RUN apt-get install -y \
    python-is-python3 \
    python3-gst-1.0 \
    python3-pip

RUN pip install\
    v4l2py \
    opencv-contrib-python \
    pyserial \
    scipy \
    pyside6 

RUN pip install --upgrade numpy==1.23.5

###### Install ROS Dependencies
RUN apt install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-imu-plugin

###### Install Dependencies
RUN  apt install -y \
    net-tools \
    ntpdate \
    v4l-utils \
    terminator \
    libx11-xcb1 libxcb-util1 libxcb-render0 libxcb-shape0 \
    libxcb-xfixes0 libxcb-keysyms1 libxcb-image0 libxcb-randr0 \
    libxcb-xtest0 libxcb-cursor0 xvfb