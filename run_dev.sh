
# TODO: only delete pixelation containers
docker ps --filter status=exited -q | xargs docker rm

# TODO: check if image is `pixelization` or not
CONTAINER_ID=$(docker ps --format '{{.ID}},{{.Names}}' | grep "^\S*,pixelization\-dev$" | head -1 | awk -F ',' '{print $1}')

#echo "BEGIN DBG"
#echo $CONTAINER_ID
#echo "END DBG"

if [[ -z $CONTAINER_ID ]]; then
  echo "Spawning new container"
  docker run --privileged \
       -v $HOME/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
       -e NVIDIA_VISIBLE_DEVICES=all \
       --network host \
       -v /dev/*:/dev/* \
       -v /etc/localtime:/etc/localtime:ro \
       --runtime nvidia \
       --workdir /workspaces/isaac_ros-dev \
       --name 'pixelization-dev' \
       -it pixelization /bin/bash
else
  echo "Attaching to existing container: $CONTAINER_ID"
  docker container attach $CONTAINER_ID
fi