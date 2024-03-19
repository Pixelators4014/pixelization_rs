# Use the specified Isaac ROS base image
FROM nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_b7e1ed6c02a6fa3c1c7392479291c035

# Run the required commands
RUN apt update && apt install -y \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-isaac-ros-yolov8 \
    ros-humble-isaac-ros-tensor-rt \
    ros-humble-isaac-ros-dnn-image-encoder \
    ros-humble-isaac-ros-detectnet \
    ros-humble-isaac-ros-triton \
    ros-humble-isaac-ros-apriltag \
    libclang-dev python3-pip python3-vcstool

# Install rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
RUN cargo install cargo-ament-build
RUN pip install git+https://github.com/colcon/colcon-cargo.git git+https://github.com/colcon/colcon-ros-cargo.git

# Set the working directory to the isaac_ros-dev workspace
WORKDIR /workspaces/isaac_ros-dev

# Build the ROS workspace
# RUN /bin/bash -c '. /opt/ros/humble/setup.bash; colcon build --symlink-install'
RUN colcon build --symlink-install
RUN echo "source /workspaces/isaac_ros-dev/install/setup.bash" >> ~/.bashrc