FROM ghcr.io/soham2560/humble-garden:latest

# Add the following labels
LABEL org.opencontainers.image.description="ROS2 Humble + ROS2 Garden + ROS2 Control"
LABEL org.opencontainers.image.title="humble-garden-control"
LABEL org.opencontainers.image.vendor="Soham Patil"
LABEL org.opencontainers.image.source="https://github.com/soham2560/DockerForROS2Development"
LABEL maintainer="sohampatil45939@gmail.com"
LABEL org.opencontainers.image.licenses="MIT"

# handle default shell
SHELL ["/bin/bash", "-c"]

USER root

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

USER container_user