# Use the specified Isaac ROS base image
FROM isaac_ros_dev-aarch64

# Run the required commands
RUN apt update \
    && apt install -y \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-isaac-ros-yolov8 \
    ros-humble-isaac-ros-tensor-rt \
    ros-humble-isaac-ros-dnn-image-encoder \
    ros-humble-isaac-ros-detectnet \
    ros-humble-isaac-ros-triton \
    ros-humble-isaac-ros-apriltag \
    libclang-dev python3-pip python3-vcstool \
    && apt clean && rm -rf /var/lib/apt/lists/*

# Set the working directory to the isaac_ros-dev workspace
WORKDIR /workspaces/isaac_ros-dev/src

# Install rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
RUN . ~/.cargo/env && cargo install cargo-ament-build
RUN pip install git+https://github.com/colcon/colcon-cargo.git git+https://github.com/colcon/colcon-ros-cargo.git

# Copy files
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common --depth 1
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag --depth 1
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection --depth 1
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam --depth 1
RUN git clone https://github.com/ros2-rust/ros2_rust --depth 1
RUN git clone https://github.com/IntelRealSense/realsense-ros --depth 1
COPY . pixelization_rs

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
RUN vcs import src < src/ros2_rust/ros2_rust_humble.repos
RUN . /opt/ros/humble/setup.bash && ~/.cargo/env && colcon build --symlink-install --packages-up-to pixelization_rs
RUN echo 'source ~/.cargo/env' >> $HOME/.bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> $HOME/.bashrc
RUN echo 'source /workspaces/isaac_ros-dev/install/setup.bash' >> $HOME/.bashrc

# # TODO: Add the entrypoint
CMD [ "/bin/bash", "-c", "ros2 launch pixelization_rs run.launch.py" ]
