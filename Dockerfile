# ROS Noetic on Ubuntu 20.04
FROM ubuntu:focal
SHELL ["/bin/bash", "-c"]

RUN echo "We are building ROS Noetic image on Ubuntu 20.04."
ARG DEBIAN_FRONTEND=noninteractive
RUN mkdir -p /home/ros/src
WORKDIR /home/ros
ENV TZ=Europe/Istanbul
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

#RUN sed -i -E -e "s:archive\.ubuntu\.com/ubuntu:mirror.verinomi.com/ubuntu/ubuntu-archive:g" /etc/apt/sources.list
#RUN sed -i -E -e "s:http:https:g" /etc/apt/sources.list
#RUN sed -i -E -e "s:deb https:deb [trusted=yes] https:g" /etc/apt/sources.list
#RUN cat /etc/apt/sources.list
RUN apt-get update && apt-get -y -q install \
        gnupg \
        apt-utils

ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=DontWarn
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && apt-get -y -q install \
        ros-noetic-desktop-full \
        ros-noetic-moveit \
        python3-osrf-pycommon \
        python3-catkin-tools \
        ros-noetic-controller-manager \
        ros-noetic-joint-trajectory-controller \
        ros-noetic-rqt-joint-trajectory-controller \
        ros-noetic-effort-controllers

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
	python3-rosdep \
        python3-pip

RUN apt-get install -y locales locales-all

SHELL ["/bin/bash", "-c"]

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility

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
ENV LC_ALL en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8

RUN pip install imgaug

RUN /opt/ros/noetic/bin/catkin_make
RUN /opt/ros/noetic/bin/catkin_make install

#RUN apt-get purge whoopsie libwhoopsie0

RUN dbus-uuidgen > /var/lib/dbus/machine-id
RUN mkdir -p /var/run/dbus
COPY NetworkManager.conf /usr/share/dbus-1/system.d/org.freedesktop.NetworkManager.conf

RUN mkdir -p /home/ros/.gazebo/models
COPY simulation/srvt-ros/srvt_ros/model /home/ros/.gazebo/models/

RUN groupadd -r ros && useradd -u 1001 -r -g ros ros
RUN chown -R ros /home/ros

COPY robosimit_ep.sh /home/ros/
RUN chmod +x /home/ros/robosimit_ep.sh

COPY robosimit_ep_copy.sh /home/ros/
RUN chmod +x /home/ros/robosimit_ep.sh
RUN chmod +x /home/ros/catkin_ws/src/srvt-ros/srvt_moveit/src/*.py
USER ros

ENTRYPOINT ["/home/ros/robosimit_ep.sh"]
