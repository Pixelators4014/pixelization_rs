docker run -it --rm \
    --privileged \
    --network host \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /etc/localtime:/etc/localtime:ro \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /dev/*:/dev/* \
    -v /etc/localtime:/etc/localtime:ro \
    --runtime nvidia \
     pixelization /bin/bash $@
