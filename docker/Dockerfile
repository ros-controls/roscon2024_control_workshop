ARG ROS_DISTRO="rolling"
FROM ros:${ROS_DISTRO}

RUN apt-get update && apt-get install -y git tig vim build-essential && \
    rm -rf /var/lib/apt/lists/*

# tiago-related deps
RUN apt update && \
    apt install -y ros-${ROS_DISTRO}-launch-param-builder ros-${ROS_DISTRO}-rqt-plot && \
    rm -rf /var/lib/apt/lists/*

COPY . /root/ws/src
WORKDIR /root/ws/src
RUN cd /root/ws/src && vcs-import < roscon2024_control_workshop.repos --recursive
RUN apt update && rosdep update && rosdep install --from-path /root/ws/src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/*
RUN  /bin/bash -c "source /opt/ros/rolling/setup.bash && cd /root/ws && colcon build --symlink-install"
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

RUN echo "alias s='source install/setup.bash'" >> /root/.bashrc
RUN echo "alias cb='colcon build --symlink-install'" >> /root/.bashrc
RUN echo "source /root/ws/install/setup.bash" >> /root/.bashrc
WORKDIR /root/ws

