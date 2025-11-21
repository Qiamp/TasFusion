# Final stable Dockerfile for ROS Noetic Development.
# This version is based on the complete ros:noetic-desktop-full image
# to ensure a stable base with compatible versions of PCL, Eigen, etc.

FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

# --- INSTALL MODERN CMAKE ---
# Required by gtsam_points, which needs CMake >= 3.22.
ENV CMAKE_VERSION=3.27.5
RUN apt-get update && apt-get install -y --no-install-recommends wget && \
    wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-x86_64.sh && \
    chmod +x cmake-${CMAKE_VERSION}-linux-x86_64.sh && \
    ./cmake-${CMAKE_VERSION}-linux-x86_64.sh --skip-license --prefix=/usr/local && \
    rm cmake-${CMAKE_VERSION}-linux-x86_64.sh && \
    apt-get remove --purge -y wget && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# 1. Install system dependencies.
# Even if desktop-full includes them, listing them explicitly ensures they are present.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git \
    libeigen3-dev \
    libceres-dev \
    libtbb-dev libgflags-dev libgoogle-glog-dev \
    libglm-dev libglfw3-dev libpng-dev \
    ros-noetic-roslint \
    && rm -rf /var/lib/apt/lists/*

# 2. Build GTSAM and gtsam_points from source.
# Critical Fix: Force GTSAM to use the system's pre-installed and compatible Eigen version
# to prevent version mismatch errors during the final catkin_make stage.
WORKDIR /root/third_party
RUN git clone https://github.com/borglab/gtsam.git && \
    cd gtsam && \
    git checkout release/4.3a0 && \
    mkdir build && cd build && \
    cmake .. \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF && \
    make -j$(nproc) && \
    make install

# WORKDIR /root/third_party
# RUN git clone https://github.com/koide3/gtsam_points.git && \
#     cd gtsam_points && \
#     mkdir build && cd build && \
#     cmake .. -DCMAKE_BUILD_TYPE=Release && \
#     make -j$(nproc) && \
#     make install

# --- Update the dynamic linker cache ---
RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/gtsam.conf && ldconfig

# --- NEW: Automatically source environment in all terminals ---
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/catkin_ws/devel/setup.bash ]; then" >> /root/.bashrc && \
    echo "  source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "fi" >> /root/.bashrc

# 3. Configure the ROS Workspace.
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
RUN source /opt/ros/noetic/setup.bash && rosdep update

# 4. Set up the entrypoint script.
COPY ./src/.docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]