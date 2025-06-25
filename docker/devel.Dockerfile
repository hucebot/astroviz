FROM nvidia/opengl:1.2-glvnd-devel-ubuntu22.04
ENV ROS_DISTRO=humble

ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=0

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="Europe/Paris"

LABEL maintainer="clemente.donosok@gmail.com"

RUN apt-get update && apt-get upgrade -y

###### Install ROS2
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get upgrade -y
RUN apt install ros-${ROS_DISTRO}-desktop-full -y
RUN apt install ros-dev-tools -y
RUN rosdep init && rosdep update

###### Install Python Dependencies
RUN apt-get install -y \
    python-is-python3 \
    python3-gst-1.0 \
    python3-pip

RUN pip install \
    v4l2py \
    opencv-contrib-python \
    pyserial \
    scipy

RUN pip install PyQt6 PyQt6-WebEngine


RUN apt-get install -y \
    libgl1-mesa-dev \
    libnss3 \
    libxcomposite1 \
    libxrandr2 \
    libxcursor1 \
    libxdamage1 \
    libxtst6 \
    libglib2.0-0 \
    libgtk-3-0


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

    
RUN pip install shapely cython numpy pyshp six 
RUN pip install shapely --no-binary shapely
RUN apt-get install -y libproj-dev libgeos-dev python3-gi-cairo
RUN pip install cartopy
RUN pip install folium
RUN apt install -y libasound2
RUN apt-get update && apt-get install -y firefox


RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
