mkdir -p ./usr/bin/
mkdir -p ./tmp/
mkdir -p ./usr/local/cuda-11.4/targets/aarch64-linux/lib/
mkdir -p ./usr/local/cuda-11.4/targets/aarch64-linux/include/
mkdir -p ./usr/lib/aarch64-linux-gnu/
mkdir -p ./usr/src/
mkdir -p ./opt/nvidia/
mkdir -p ./usr/share/
cp -r /usr/bin/tegrastats ./usr/bin/
cp -r /tmp/argus_socket ./tmp/
cp /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11 ./usr/local/cuda-11.4/targets/aarch64-linux/lib/
cp /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11 ./usr/local/cuda-11.4/targets/aarch64-linux/lib/
cp /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10 ./usr/local/cuda-11.4/targets/aarch64-linux/lib/
cp /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcufft.so.10 ./usr/local/cuda-11.4/targets/aarch64-linux/lib/
cp /usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so ./usr/local/cuda-11.4/targets/aarch64-linux/lib/
cp /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4 ./usr/local/cuda-11.4/targets/aarch64-linux/lib/
cp /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1 ./usr/local/cuda-11.4/targets/aarch64-linux/lib/
cp /usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h ./usr/local/cuda-11.4/targets/aarch64-linux/include/
cp -r /usr/lib/aarch64-linux-gnu/tegra ./usr/lib/aarch64-linux-gnu/
cp -r /usr/src/jetson_multimedia_api ./usr/src/
cp -r /opt/nvidia/nsight-systems-cli ./opt/nvidia/
cp -r /opt/nvidia/vpi2 ./opt/nvidia/
cp -r /usr/share/vpi2 ./usr/share/
docker build -t pixelization .
