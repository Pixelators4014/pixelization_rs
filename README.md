# Pixelization

A ROS2 node that collects data from VSLAM and april tags and processes it for the RobotRIO.

## Requirements

The host system must be an Orin Nano Developer Kit.

It must have docker configured and launching the `run_dev` script from isaac_ros_common should not fail.

Ensure dependencies are up to date:
```shell
sudo apt install libnvvpi3 vpi3-dev vpi3-samples
```

