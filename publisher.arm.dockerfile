FROM dustynv/ros:humble-ros-base-l4t-r36.2.0 AS build

ENV DEBIAN_FRONTENT=nointeractive \
SHELL=/bin/bash


SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake \
    python3-pil \
    python3-opencv \
    python3-numpy \
    python3-matplotlib \
    libffi-dev \
    # libopenlas-dev \
    # zliblg-dev \
    libjpeg-dev \
    liblapack-dev \
    # gfrotran \
    python3-pip \
    python3-dev \
    g++ \
    python3-setuptools \
    python3-tk \
    gnupg \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean
    
WORKDIR /tmp

RUN pip3 install --no-cache-dir \
    einops \
    timm \
    visdom \
    easydict \
    scipy \
    scikit-learn==1.3.2 \
    mock
    


# install python packages
RUN pip3 install -U \
    argcomplete \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures

RUN pip3 freeze | grep pytest \
    && python3 -m pytest --version

# # bootstrap rosdep
# RUN rosdep init \
#     && rosdep update

# # setup colcon mixin and metadata
# RUN colcon mixin add default \
#       https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
#     colcon mixin update && \
#     colcon metadata add default \
#       https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
#     colcon metadata update

# clone source
ENV ROS2_WS /opt/ros2_ws
RUN mkdir -p $ROS2_WS/src
WORKDIR $ROS2_WS

# build source
RUN colcon \
    build \
    --cmake-args \
      -DSECURITY=ON --no-warn-unused-cli \
    --symlink-install

ARG ROS_DISTRO=humble

RUN mkdir -p /home/user/src/

COPY src/ /home/user/src/


RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash"

CMD ["python3", "/home/user/src/mock_image_publisher.py"]