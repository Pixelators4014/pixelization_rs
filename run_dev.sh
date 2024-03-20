docker run --privileged --network host -v /dev/*:/dev/* -v /etc/localtime:/etc/localtime:ro --runtime nvidia -it pixelization /bin/bash
