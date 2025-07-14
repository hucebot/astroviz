FROM nvidia/opengl:1.2-glvnd-devel-ubuntu22.04
LABEL maintainer="clemente.donosok@gmail.com"

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Europe/Paris \
    ROS_DISTRO=humble \
    DISPLAY=:0 \
    LIBGL_ALWAYS_INDIRECT=0

SHELL ["/bin/bash", "-c"]

##### Install dependencies for the base system
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      curl gnupg2 lsb-release ca-certificates \
      net-tools ntpdate v4l-utils terminator xvfb firefox \
      libgl1-mesa-dev libnss3 libx11-xcb1 libxcomposite1 libxrandr2 \
      libxcursor1 libxdamage1 libxtst6 libglib2.0-0 libgtk-3-0 \
      libxcb-util1 libxcb-render0 libxcb-shape0 libxcb-xfixes0 \
      libxcb-keysyms1 libxcb-image0 libxcb-randr0 libxcb-xtest0 \
      libxcb-cursor0 libasound2 libproj-dev libgeos-dev python3-gi-cairo && \
    rm -rf /var/lib/apt/lists/*

##### Install python dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python-is-python3 python3-opengl python3-pip python3-gst-1.0 && \
    python3 -m pip install --upgrade pip numpy==1.23.5 && \
    python3 -m pip install --no-cache-dir \
      v4l2py opencv-contrib-python pyserial scipy \
      PyQt6 PyQt6-WebEngine pyqtgraph termcolor ping3 \
      shapely cython pyshp six cartopy folium  urdfpy && \
    rm -rf /var/lib/apt/lists/* ~/.cache/pip

##### Install ROS 2 Humble
RUN export ROS_APT_SRC_VER=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SRC_VER}/ros2-apt-source_${ROS_APT_SRC_VER}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-desktop-full \
      ros-dev-tools \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-rviz-imu-plugin && \
    rosdep init && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/* /tmp/ros2-apt-source.deb

# Fuente automática de ROS en cada terminal
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc

RUN pip install --upgrade "pybind11>=2.12" "numpy<2"
RUN pip install urdfpy 

RUN mkdir -p /ros2_ws/src

RUN  apt update
RUN  apt -q -qq update && apt install -y --allow-unauthenticated \
  gstreamer1.0-plugins-good \ 
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-rtp \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-tools \
  python3-gst-1.0 \
  libgstreamer1.0-dev \
  net-tools curl

RUN apt-get update && apt-get install -y --no-install-recommends \
      gstreamer1.0-plugins-base && \
    # genera la registry.bin en build para no arrancar el plugin-scanner en runtime
    gst-inspect-1.0 > /dev/null && \
    rm -rf /var/lib/apt/lists/*


CMD ["bash"]
