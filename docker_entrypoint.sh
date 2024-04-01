ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py &
cd /workspaces/isaac_ros-dev/src/pixelization_rs/docker_entrypoint.sh
cargo run -r

wait