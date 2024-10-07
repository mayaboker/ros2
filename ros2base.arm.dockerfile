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
    libopenblas-dev \
    zlib1g-dev \
    libjpeg-dev \
    liblapack-dev \
    gfortran \
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

RUN pip3 install --no-cache-dir \
    opencv-python \
    setuptools \
    wheel \
    pyrallis \
    cython \
    numpy==1.24.0 \
    jpeg4py
    
WORKDIR /tmp

RUN pip3 install --no-cache-dir \
    einops \
    timm \
    visdom \
    easydict \
    scipy \
    scikit-learn==1.3.2 \
    mock
    
WORKDIR  /home

RUN pip3 install "pandas>=1.1.4" \
    "py_cpuinfo" \
    "ultralytics-thop>=2.0.1"

# missing only dependency - installation of seaborn. didn't add because it shouldn't be required, and contains numpy, which causses issues.
RUN pip3 install ultralytics --no-deps

COPY packages /tmp/packages/

RUN pip3 install /tmp/packages/*

# clone source
ENV ROS2_WS=/opt/ros2_ws
RUN mkdir -p $ROS2_WS/src
WORKDIR $ROS2_WS

ENV LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1

# build source
RUN colcon \
    build \
    --cmake-args \
      -DSECURITY=ON --no-warn-unused-cli \
    --symlink-install

ARG ROS_DISTRO=humble

RUN mkdir -p /home/user/src/

COPY src/ /home/user/src/

RUN mkdir -p /home/user/weights/

COPY weights/ /home/user/weights/

ENV PYTHONPATH=$PYTHONPATH:/opt/ros/humble/install/lib/python3.8/site-packages

ENV PATH=$PATH:/opt/ros/humble/install/bin:/usr/local/cuda/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

ENV PKG_CONFIG_PATH=/ros_deep_learning/install/ros_deep_learning/lib/aarch64-linux-gnu/pkgconfig:/ros_deep_learning/install/ros_deep_learning/lib/pkgconfig:/opt/ros/humble/install/lib/aarch64-linux-gnu/pkgconfig:/opt/ros/humble/install/lib/pkgconfig

ENV ROS_PACKAGE_PATH=/ros_deep_learning/install/ros_deep_learning/share

ENV LD_LIBRARY_PATH=/opt/ros/humble/install/lib:/usr/local/cuda/lib64:/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu/:

ENV CUDA_HOME=/usr/local/cuda

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash"

# CMD ["python3", "/home/user/src/INSERT_FILE_NAME_HERE.py"]
