FROM ghcr.io/soham2560/humble:latest

LABEL org.opencontainers.image.description="ROS2 Humble + LIDAR Camera Calibration"
LABEL org.opencontainers.image.title="humble-lidar-cam"
LABEL org.opencontainers.image.vendor="Soham Patil"
LABEL org.opencontainers.image.source="https://github.com/soham2560/LiDAR_Camera_Calibration"
LABEL maintainer="sohampatil45939@gmail.com"
LABEL org.opencontainers.image.licenses="MIT"

USER root

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    curl \
    apt-transport-https \
    lsb-release

RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null
    
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/librealsense.list

COPY rosPkgs.list /tmp/rosPkgs.list
RUN apt-get update \
   && apt-get -y install --no-install-recommends $(cat /tmp/rosPkgs.list) \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
   
RUN apt-get update \
   && apt-get install --no-install-recommends -y software-properties-common gnupg ca-certificates \
   && add-apt-repository ppa:apt-fast/stable \
   && apt-get update \
   && apt-get install --no-install-recommends -y apt-fast \
   && apt-fast clean \
   && rm -rf /var/lib/apt/lists/*

RUN echo debconf apt-fast/maxdownloads string 16 | debconf-set-selections && \
   echo debconf apt-fast/dlflag boolean true | debconf-set-selections && \
   echo debconf apt-fast/aptmanager string apt-get | debconf-set-selections

RUN apt-fast update \
   && apt-fast upgrade -y \
   && apt-fast install --no-install-recommends -y \
      wget nano build-essential clang lld libomp-dev \
      git cmake libeigen3-dev ca-certificates \
      libtbb-dev libboost-all-dev libgtest-dev libmetis-dev \
      libglm-dev libglfw3-dev libpng-dev libjpeg-dev \
   && apt-fast clean \
   && rm -rf /var/lib/apt/lists/*

WORKDIR /root
RUN git clone https://github.com/borglab/gtsam.git \
   && cd gtsam \
   && git checkout 4.2a9 \
   && mkdir build && cd build \
   && cmake .. \
   -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
   -DGTSAM_BUILD_TESTS=OFF \
   -DGTSAM_WITH_TBB=OFF \
   -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
   -DGTSAM_USE_SYSTEM_EIGEN=ON \
   -DGTSAM_USE_SYSTEM_METIS=ON \
   && make -j$(nproc) \
   && make install \
   && rm -rf /root/gtsam

WORKDIR /root
RUN git clone https://github.com/koide3/iridescence.git --recursive \
   && mkdir iridescence/build && cd iridescence/build \
   && cmake .. \
   && make -j$(nproc) \
   && make install \
   && rm -rf /root/iridescence

   RUN apt-fast update \
   && apt-fast install -y --no-install-recommends \
   libomp-dev libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev \
   && apt-fast clean \
   && rm -rf /var/lib/apt/lists/*
 
WORKDIR /root
RUN git clone https://github.com/ceres-solver/ceres-solver \
   && cd ceres-solver \
   && git checkout e47a42c2957951c9fafcca9995d9927e15557069 \
   && mkdir build \
   && cd build \
   && cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DUSE_CUDA=OFF \
   && make -j$(nproc) \
   && make install \
   && rm -rf /root/ceres-solver

RUN apt-fast update \
   && apt-fast install -y --no-install-recommends \
   python3-pip python3-numpy python3-torch python3-torchvision python3-matplotlib python3-opencv \
   && apt-fast clean \
   && rm -rf /var/lib/apt/lists/*

WORKDIR /root
RUN git clone https://github.com/magicleap/SuperGluePretrainedNetwork /home/container_user/SuperGluePretrainedNetwork

RUN git clone https://github.com/Microsoft/vcpkg.git /vcpkg
WORKDIR /vcpkg
RUN ./bootstrap-vcpkg.sh
RUN ./vcpkg integrate install
RUN ./vcpkg install sophus

WORKDIR /ros2_ws/src
USER container_user