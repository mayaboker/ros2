FROM ros:humble-ros-core

ARG ROS_DISTRO=humble

RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-vision-msgs \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/user/src/

RUN pip3 install --no-cache-dir opencv-python-headless

COPY src/ /home/user/src/

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash"

CMD ["python3", "/home/user/src/bbox_node.py"]