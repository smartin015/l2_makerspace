ARG L2PKG=l2_storage
ARG L2NODES=node.py
FROM l2base-pre
FROM l2base-post

RUN wget https://developer.download.nvidia.com/compute/cuda/11.0.3/local_installers/cuda_11.0.3_450.51.06_linux.run && sh cuda_11.0.3_450.51.06_linux.run && \
    export PATH=/usr/local/cuda-11.0/bin${PATH:+:${PATH}} && \
    export LD_LIBRARY_PATH=/usr/local/cuda-11.0/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}


# Dependencies for downloading & building librealsense2                               
#RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3-pip wget unzip cmake libssl-dev libusb-1.0-0-dev pkg-config python3 python3-dev libx11-dev libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev && rm -rf /var/lib/apt/lists/*                                                                                    
                                                                                            
# Takes awhile, try to avoid disrupting the cache with earlier commands           
#RUN wget https://github.com/IntelRealSense/librealsense/archive/master.zip && \   
#        unzip master.zip && cd librealsense-master && mkdir build && cd build && \
#        CUDACXX=$(which nvcc) PATH=${PATH}:/usr/local/cuda/bin \         
#        LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda/lib64 \       
#        /usr/bin/cmake ../ -DFORCE_LIBUVC=true -DBUILD_WITH_CUDA="true" \
#        -DCMAKE_BUILD_TYPE=release -DBUILD_PYTHON_BINDINGS=bool:true \
#        -DPYTHON_EXECUTABLE=$(which python3.8) && \
#        export NUM_CPU=$(nproc) && \
#        make -j$((NUM_CPU - 1)) && make install 

#RUN apt-get update && apt-get install -y --no-install-recommends python3-tk && \
#    rm -rf /var/lib/apt/lists/*

#RUN pip3 install opencv-python numba matplotlib

#ENV PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages/pyrealsense2/

COPY --from=0 /node/install /node/install


