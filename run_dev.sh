docker run --privileged \
       -v $HOME/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
       -e NVIDIA_VISIBLE_DEVICES=all \
       --network host \
       -v /dev/*:/dev/* \
       -v /etc/localtime:/etc/localtime:ro \
       --runtime nvidia \
       --workdir /workspaces/isaac_ros-dev \
       -it pixelization /bin/bash