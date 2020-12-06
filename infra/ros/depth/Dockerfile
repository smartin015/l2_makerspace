# Remember to run the container with 
#   nvidia-docker run -it --rm --volume=$(pwd):/volume rvlcuda /bin/bash
# (specifically nvidia-docker, not docker)
# so that libcuda.so is available at container run time
# 
# For display version:
# xhost +
# nvidia-docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY -e XAUTHORITY -e NVIDIA_DRIVER_CAPABILITIES=all --volume=$(pwd):/volume rvlcuda /bin/bash
FROM nvidia/cuda:11.0-devel

# Dependencies for downloading & building librealsense2
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3-pip wget unzip cmake libssl-dev libusb-1.0-0-dev pkg-config python3 python3-dev libx11-dev libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev && rm -rf /var/lib/apt/lists/*

# Takes awhile, try to avoid disrupting the cache with earlier commands
RUN wget https://github.com/IntelRealSense/librealsense/archive/master.zip && \
	unzip master.zip && cd librealsense-master && mkdir build && cd build && \
	CUDACXX=$(which nvcc) PATH=${PATH}:/usr/local/cuda/bin \
	LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda/lib64 \
	/usr/bin/cmake ../ -DFORCE_LIBUVC=true -DBUILD_WITH_CUDA="true" \
	-DCMAKE_BUILD_TYPE=release -DBUILD_PYTHON_BINDINGS=bool:true \
	-DPYTHON_EXECUTABLE=$(which python3.8) && \
	export NUM_CPU=$(nproc) && \
	make -j$((NUM_CPU - 1)) && make install 

RUN apt-get update && apt-get install -y --no-install-recommends python3-tk && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install opencv-python numba matplotlib

ENV PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages/pyrealsense2/
