sudo apt install -y git libclang-dev python3-pip python3-vcstool # libclang-dev is required by bindgen
$ curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain none
# Install these plugins for cargo and colcon:
cargo install cargo-ament-build
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git
