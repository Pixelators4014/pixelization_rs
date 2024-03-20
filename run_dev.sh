docker run --privileged --network host -v /dev/*:/dev/* -v /etc/localtime:/etc/localtime:ro --runtime nvidia --user="admin" -it pixelization /bin/bash
