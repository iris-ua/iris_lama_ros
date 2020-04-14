# This is an auto generated Dockerfile for ros:ros-core
# generated from docker_images_ros2/create_ros_core_image.Dockerfile.em
FROM ubuntu:bionic

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu bionic main" > /etc/apt/sources.list.d/ros2-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO dashing
# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install python packages
RUN pip3 install -U \
    argcomplete

# install ros2 packages
RUN apt-get update && apt-get install -y \
    ros-dashing-ros-core=0.7.3-1* \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
#COPY ./ros_entrypoint.sh /

#ENTRYPOINT ["/ros_entrypoint.sh"]
# https://github.com/ActiveIntelligentSystemsLab/ros2-docker/blob/master/docker/ros_entrypoint.sh

RUN apt update && apt install -y curl wget tmux nano python3-colcon-common-extensions \
 google-mock libceres-dev liblua5.3-dev libboost-dev libboost-iostreams-dev \
 libprotobuf-dev protobuf-compiler libcairo2-dev libpcl-dev python3-sphinx
#curl -sSL http://get.gazebosim.org | sh
RUN apt install -y ros-dashing-gazebo-* ros-dashing-cartographer ros-dashing-cartographer-ros ros-dashing-navigation2 ros-dashing-nav2-bringup python3-vcstool ros-dashing-robot-state-publisher ros-dashing-rviz* ros-dashing-ros2bag ros-dashing-rosbag2-storage-default-*

RUN mkdir -p ~/turtlebot3_ws/src
RUN cd ~/turtlebot3_ws && wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos && vcs import src < turtlebot3.repos

RUN echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
RUN echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
# source ~/.bashrc

RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' > /ros_entrypoint.sh
RUN echo /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
#ENTRYPOINT ["/ros_entrypoint.sh"]

#RUN bash ros_entrypoint.sh && cd ~/turtlebot3_ws && colcon build --symlink-install

CMD ["bash"]