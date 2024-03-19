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

# Install rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
RUN /bin/bash -c 'source ~/.cargo/env; cargo install cargo-ament-build'
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
COPY /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11 /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11

WORKDIR /workspaces/isaac_ros-dev

# Build the ROS workspace
RUN /bin/bash -c 'vcs import src < src/ros2_rust/ros2_rust_humble.repos; source /opt/ros/humble/setup.bash; source ~/.cargo/env; colcon build --symlink-install --packages-up-to pixelization_rs'

# # TODO: Add the entrypoint
CMD [ "/bin/bash", "-c", "source ~/.cargo/env; source /opt/ros/humble/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash && ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py" ]
