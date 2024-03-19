# Use the specified Isaac ROS base image
FROM nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_b7e1ed6c02a6fa3c1c7392479291c035

# Run the required commands
RUN apt update \
    && sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
    && apt install -y \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-isaac-ros-yolov8 \
    ros-humble-isaac-ros-tensor-rt \
    ros-humble-isaac-ros-dnn-image-encoder \
    ros-humble-isaac-ros-detectnet \
    ros-humble-isaac-ros-triton \
    ros-humble-isaac-ros-apriltag \
    libclang-dev python3-pip python3-vcstool \
    librealsense2-utils librealsense2-dev \
    && apt clean && rm -rf /var/lib/apt/lists/*

# Install rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"
RUN cargo install cargo-ament-build
RUN pip install git+https://github.com/colcon/colcon-cargo.git git+https://github.com/colcon/colcon-ros-cargo.git

# Set the working directory to the isaac_ros-dev workspace
WORKDIR /workspaces/isaac_ros-dev/src

# Copy files
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
RUN git clone https://github.com/ros2-rust/ros2_rust
RUN git clone https://github.com/IntelRealSense/realsense-ros
COPY . pixelization_rs/

# Copy files
COPY /usr/bin/tegrastats /usr/bin/tegrastats
COPY /tmp/argus_socket /tmp/argus_socket
COPY /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11 /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11
COPY /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11 /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11
COPY /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10 /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10
COPY /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcufft.so.10 /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcufft.so.10
COPY /usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so /usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so
COPY /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4 /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4
COPY /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1 /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1
COPY /usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h /usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h
COPY /usr/lib/aarch64-linux-gnu/tegra /usr/lib/aarch64-linux-gnu/tegra
COPY /usr/src/jetson_multimedia_api /usr/src/jetson_multimedia_api
COPY /opt/nvidia/nsight-systems-cli /opt/nvidia/nsight-systems-cli
COPY /opt/nvidia/vpi2 /opt/nvidia/vpi2
COPY /usr/share/vpi2 /usr/share/vpi2

WORKDIR /workspaces/isaac_ros-dev

# Build the ROS workspace
RUN /bin/bash -c 'vcs import src < src/ros2_rust/ros2_rust_humble.repos; source /opt/ros/humble/setup.bash; colcon build --symlink-install'

# # TODO: Add the entrypoint
CMD [ "/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash && ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py" ]
