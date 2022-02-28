ARG ROS_DISTRO=foxy
FROM ros:${ROS_DISTRO}-ros-base
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN apt update
RUN apt install -y python3-rosdep2
RUN rosdep update
RUN apt update
RUN apt -y dist-upgrade
RUN apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt install python3-colcon-common-extensions python3-vcstool
RUN apt update \
  && apt install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 libogre-1.9-dev\
  && rm -rf /var/lib/apt/lists/*# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
RUN DEBIAN_FRONTEND=noninteractive \
	apt update && \
	apt install -y mesa-utils libgl1-mesa-glx libglu1-mesa-dev freeglut3-dev mesa-common-dev libopencv-dev python3-opencv python3-tk 
RUN DEBIAN_FRONTEND=noninteractive \
	apt update && \
	apt install -y ros-$ROS_DISTRO-ros2-control

ARG USER=robot
ARG PASSWORD=robot
ARG UID=1000
ARG GID=1000
ARG DOMAIN_ID=0
ENV UID=${UID}
ENV GID=${GID}
ENV USER=${USER}
RUN groupadd -g "$GID" "$USER"  && \
    useradd -m -u "$UID" -g "$GID" --shell $(which bash) "$USER" -G sudo && \
    echo "$USER:$PASSWORD" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /etc/bash.bashrc
RUN echo "export _colcon_cd_root=~/ros2_install" >> /etc/bash.bashrc
RUN echo "export ROS_DOMAIN_ID=${DOMAIN_ID}" >> /etc/bash.bashrc

USER $USER 
RUN rosdep update

RUN mkdir -p /home/"$USER"/ws_moveit2/src
RUN cd /home/"$USER"/ws_moveit2/src && git clone https://github.com/ros-planning/moveit2.git -b foxy && vcs import < moveit2/moveit2.repos && rosdep install -r --from-paths . --ignore-src --rosdistro foxy -y

#RUN cd /home/"$USER"/ws_moveit2/src/geometric_shapes && git checkout foxy
RUN cd /home/$USER/ws_moveit2 && . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN echo "source /home/$USER/ws_moveit2/install/setup.bash" >> /home/$USER/.bashrc

RUN mkdir -p /home/"$USER"/ros_ws/src
RUN cd /home/"$USER"/ros_ws && colcon build
RUN echo "source /home/$USER/ros_ws/install/setup.bash" >> /home/$USER/.bashrc

WORKDIR /home/$USER/ros_ws

CMD /bin/bash
