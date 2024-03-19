mkdir -p ./usr/local/cuda-11.4/targets/aarch64-linux/lib/
cp /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11 ./usr/local/cuda-11.4/targets/aarch64-linux/lib/
docker build -t pixelization .
