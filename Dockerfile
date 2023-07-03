FROM ubuntu:20.04

SHELL ["/bin/bash", "-ci"]
ENV DEBIAN_FRONTEND noninteractive

WORKDIR /workspace/ros_ws

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

CMD ["/bin/bash"]