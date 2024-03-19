sudo apt-get update
sudo apt-get install -y ros-humble-isaac-ros-visual-slam \
ros-humble-isaac-ros-yolov8 ros-humble-isaac-ros-tensor-rt ros-humble-isaac-ros-dnn-image-encoder \
ros-humble-isaac-ros-detectnet ros-humble-isaac-ros-triton ros-humble-isaac-ros-dnn-image-encoder \
ros-humble-isaac-ros-apriltag
sudo apt-get autoremove -y

# Rust setup
sudo apt install -y git libclang-dev python3-pip python3-vcstool # libclang-dev is required by bindgen
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain none
# Install these plugins for cargo and colcon:
cargo install cargo-ament-build
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git

# Colcon build + install
colcon build --symlink-install
source install/setup.bash

# More rust setup
vcs import src < src/ros2_rust/ros2_rust_humble.repos
. /opt/ros/humble/setup.sh
