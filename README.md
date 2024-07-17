# Pixelization

A ROS2 node that collects data from VSLAM and april tags and processes it for the RobotRIO.

## Requirements

The host system must be an Orin Nano Developer Kit.
## Full setup
Ensure `curl`, `git`, and `tar` are installed.

If docker hasn't been properly configured, follow the docker instructions on this page: https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/index.html#jetson-platforms

If docker has been configured, or you're done with that step, configure power settings via the commands on this page: https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/index.html#jetson-platforms (just the power/performance specific ones)

Follow everything on this page: https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html (not the other pages though)

Clone isaac ros common, if not cloned already.
```shell
cd ${ISAAC_ROS_WS}/src && \
   git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
```
Now wait for the containers to build, if it is successful, you should see a shell in the docker container.
```shell
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

Exit the shell and run (if not cloned already)
```shell
cd ${ISAAC_ROS_WS}/src && \
   git clone https://github.com/Pixelators4014/pixelization_rs.git
```
Now, finally, to build the container, run:
```shell
cd ${ISAAC_ROS_WS}/src/pixelization_rs && \
source build.sh
```

Now, to get attach terminal in the container (which will be spun up if needed), simply run `source run_dev.sh`.

Note that devcontainers are supported, and are suggested if you like autocomplete,
just open the file [.devcontainer/devcontainer.json](.devcontainer/devcontainer.json) and your IDE should take care of the rest.
