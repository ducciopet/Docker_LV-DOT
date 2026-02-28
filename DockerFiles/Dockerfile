# ============================================================
# CUDA BASE (Ubuntu 22.04 + GPU)
# ============================================================
FROM nvidia/cuda:12.2.0-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_INDIRECT=0

# ============================================================
# BASE UBUNTU PACKAGES
# ============================================================
RUN apt-get update && apt-get install -y \
    apt-transport-https \
    ca-certificates \
    software-properties-common \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    git \
    wget \
    sudo \
    nano \
    tmux \
    xterm \
    iputils-ping \
    iproute2 \
    mesa-utils \
    libgl1 \
    libglu1-mesa \
    python3-pip \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# ROS2 REPOSITORY
# ============================================================
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
> /etc/apt/sources.list.d/ros2.list

# ============================================================
# ROS2 INSTALL
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-humble-desktop-full \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-plotjuggler-ros \
    ros-humble-rviz2 \
    ros-humble-tf2-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ============================================================
# PYTHON ML DEPENDENCIES
# ============================================================
RUN apt-get update && apt-get remove -y \
    python3-sympy \
    python3-mpmath \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip setuptools wheel

# PyTorch with CUDA support (cu118) and break system packages to handle conflicts
RUN python3 -m pip install --no-cache-dir --break-system-packages \
    torch torchvision torchaudio \
    --index-url https://download.pytorch.org/whl/cu118

# Computer Vision and ML libraries
RUN python3 -m pip install --no-cache-dir --break-system-packages \
    opencv-python \
    opencv-contrib-python \
    ultralytics \
    "numpy<2" \
    scipy \
    scikit-learn \
    matplotlib \
    pillow \
    pyyaml \
    tqdm \
    requests

# ============================================================
# USER
# ============================================================
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
 && useradd -m --uid $USER_UID --gid $USER_GID -s /bin/bash $USERNAME \
 && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME
WORKDIR /home/$USERNAME

# ============================================================
# WORKSPACE
# ============================================================
RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws

SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f install/setup.bash ]; then source install/setup.bash; fi" >> ~/.bashrc
