ARG UBUNTU_VERSION=18.04
ARG CUDA_VERSION=11.3.0
FROM nvidia/cudagl:${CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION}

ENV DEBIAN_FRONTEND=noninteractive

# Setup nvidia-docker hooks
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Install required software
RUN apt update && apt install -y --no-install-recommends \
        software-properties-common \
        ca-certificates \
        build-essential \
        cmake \
        git \
        curl \
        wget \
    && apt-get -y autoremove \
    && apt-get clean \
    # cleanup
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

SHELL ["/bin/bash", "-c"]

# CUDA variables required JIT compilation
# CUDA_CACHE_PATH should be used to attach a volume for caching the JIT compilation
ENV CUDA_CACHE_MAXSIZE=2147483647
ENV CUDA_CACHE_DISABLE=0
ENV CUDA_CACHE_PATH=/root/.nv/ComputeCache

# Install ROS
ENV ROS_DISTRO=melodic
ENV PYTHON_PREFIX=python
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y ros-${ROS_DISTRO}-ros-base && \
    apt-get install -y ${PYTHON_PREFIX}-rosdep ${PYTHON_PREFIX}-rosinstall ${PYTHON_PREFIX}-rosinstall-generator && \
    apt-get install -y ${PYTHON_PREFIX}-wstool ${PYTHON_PREFIX}-catkin-pkg ${PYTHON_PREFIX}-catkin-tools && \
    apt-get install -y ros-${ROS_DISTRO}-rviz ros-${ROS_DISTRO}-rqt && \
    rosdep init && rosdep fix-permissions && rosdep update -y

# Create catkin workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

RUN mkdir -p /root/catkin_ws/src/i3dr-ros-mapping-demo

# Add package file to package folder
COPY package.xml /root/catkin_ws/src/i3dr-ros-mapping-demo/
# Install ROS dependencies
RUN apt-get update && rosdep update -y && \
    rosdep install --from-paths src -y -r --rosdistro ${ROS_DISTRO} \
    --skip-keys="${ROSDEP_SKIP_KEYS}"

# Add build files to package folder
COPY CMakeLists.txt /root/catkin_ws/src/i3dr-ros-mapping-demo/
# Build catkin workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DWITH_I3DRSGM=ON && \
    catkin build

# Copy source files to package folder
COPY config /root/catkin_ws/src/i3dr-ros-mapping-demo/config
COPY launch /root/catkin_ws/src/i3dr-ros-mapping-demo/launch

# Setup entrypoint
COPY docker/mapping-ros/entry.bash /entry.bash
RUN ["chmod", "+x", "/entry.bash"]
ENTRYPOINT ["/entry.bash"]
