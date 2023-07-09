FROM ubuntu:20.04

SHELL ["/bin/bash", "-ci"]
ENV DEBIAN_FRONTEND noninteractive

WORKDIR /workspace/

RUN apt-get update && \
    apt-get install -y \
        castxml \
        libboost-filesystem-dev \
        libboost-numpy-dev \
        libboost-program-options-dev \
        libboost-python-dev \
        libboost-serialization-dev \
        libboost-system-dev \
        libboost-test-dev \
        libeigen3-dev \
        libpcl-dev \
        libexpat1 \
        libflann-dev \
        libtriangle-dev \
        ninja-build \
        pkg-config \
        python3-pip \
        pypy3 \
        wget \
        git
RUN apt-get install curl -y
RUN mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null
# RUN apt-get update
RUN apt-get install apt-transport-https
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | tee /etc/apt/sources.list.d/librealsense.list
RUN apt-get update
RUN apt-get install librealsense2-dkms -y
RUN apt-get install librealsense2-utils -y 
RUN apt-get install librealsense2-dev -y 
RUN apt-get install librealsense2-dbg -y
RUN apt-get install cmake -y
RUN apt-get install libglfw3 libglfw3-dev -y

WORKDIR /workspace/

RUN apt-get install freeglut3-dev -y
RUN apt-get install mesa-utils

# Opencv
RUN mkdir ocv && cd ocv && git clone -b 3.4 https://github.com/opencv/opencv.git && \
	git clone -b 3.4 https://github.com/opencv/opencv_contrib.git 

RUN apt-get install libgstreamer* -y

# RUN cd ocv/opencv && mkdir build && cd build && \
# 	cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local \
# 	-D OPENCV_EXTRA_MODULES_PATH=/ocv/opencv_contrib/modules .. && \
# 	make -j$(nproc)

# RUN cd ocv/opencv/build/doc/ && make -j$(nproc) && cd .. && make install
RUN apt-get update && apt-get install -y libopencv-dev  libopencv-core4.2

RUN apt-get update && apt-get upgrade -y && apt-get dist-upgrade
RUN git clone https://github.com/IntelRealSense/librealsense.git
RUN apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev cmake -y
RUN apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at -y

# RUN bash /scripts/setup_udev_rules.sh
# RUN bash /scripts/patch-realsense-ubuntu-lts-hwe.sh
RUN apt-get install vtk7
# RUN cp /usr/bin/vtk7 /usr/bin/vtk -r
WORKDIR /workspace/librealsense
RUN mkdir build && cd build && cmake ../ -DBUILD_PCL_EXAMPLES=true && make install

WORKDIR /workspace/
RUN apt-get install libgoogle-glog-dev pcl-tools -y

WORKDIR /workspace/
CMD ["/bin/bash"]