# ROS Noetic on Ubuntu 20.04
FROM ubuntu:focal
SHELL ["/bin/bash", "-c"]

RUN echo "We are building ROS Noetic image on Ubuntu 20.04."
ARG DEBIAN_FRONTEND=noninteractive
RUN mkdir -p /home/ros/src
WORKDIR /home/ros
ENV TZ=Europe/Istanbul
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN sed -i -E -e "s:archive\.ubuntu:ubuntu.turhost:g" /etc/apt/sources.list
RUN apt-get update && apt-get -y -q install \
        gnupg \
        apt-utils

ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=DontWarn
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && apt-get -y -q install \
        ros-noetic-desktop-full

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update && \
    apt-get install -y -qq --no-install-recommends \
        mesa-utils \
        nvidia-driver-465 \
        libglvnd-dev \
        libgl1 \
        libglx0 \
        libegl1 \
        libglvnd0 \
        libgl1-mesa-dev \
        libegl1-mesa-dev \
        libxext6 \
        libx11-6 \
        libegl1 \
        libgl1-mesa-glx \
	gnome-terminal \
	python3-rosdep

SHELL ["/bin/bash", "-c"]

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility

COPY robosimit_ep.sh /home/ros/
RUN chmod +x /home/ros/robosimit_ep.sh

WORKDIR /home/ros
ENV DISPLAY :1

RUN rosdep init
RUN rosdep update
RUN mkdir -p catkin_ws/src
COPY simulation /home/ros/catkin_ws/src
WORKDIR /home/ros/catkin_ws

ENV CMAKE_PREFIX_PATH=/home/ros/catkin_ws/devel:/opt/ros/noetic
ENV PYTHONPATH=/home/ros/catkin_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages
ENV PKG_CONFIG_PATH=/home/ros/catkin_ws/devel/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig
ENV PATH=/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV CATKIN_DEVEL_PREFIX=/home/ros/catkin_ws/devel
ENV CMAKE_INSTALL_PREFIX=/home/ros/catkin_ws/install

RUN /opt/ros/noetic/bin/catkin_make
RUN /opt/ros/noetic/bin/catkin_make install

ENTRYPOINT ["/home/ros/robosimit_ep.sh"]
CMD bash
