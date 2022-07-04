FROM nvidia/cudagl:10.2-devel-ubuntu18.04

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
RUN apt-get update && apt-get -y --no-install-recommends install \
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

RUN wget https://www.baslerweb.com/fp-1615275617/media/downloads/software/pylon_software/pylon_6.2.0.21487-deb0_amd64.deb && \
    wget https://bugs.launchpad.net/~ubuntu-security-proposed/+archive/ubuntu/ppa/+build/18845128/+files/libicu55_55.1-7ubuntu0.5_amd64.deb && \
    wget http://security.ubuntu.com/ubuntu/pool/universe/x/xerces-c/libxerces-c3.1_3.1.3+debian-1_amd64.deb && \
    wget https://launchpad.net/~ubuntu-security/+archive/ubuntu/ppa/+build/15108504/+files/libpng12-0_1.2.54-1ubuntu1.1_amd64.deb && \
    wget https://github.com/i3drobotics/phobosIntegration/releases/download/v1.0.54/Phobos-1.0.54-x86_64_reducedTemplates.deb

RUN dpkg -i pylon_6.2.0.21487-deb0_amd64.deb && \
    dpkg -i libicu55_55.1-7ubuntu0.5_amd64.deb && \
    dpkg -i libxerces-c3.1_3.1.3+debian-1_amd64.deb && \
    dpkg -i libpng12-0_1.2.54-1ubuntu1.1_amd64.deb && \
    dpkg -i Phobos-1.0.54-x86_64_reducedTemplates.deb

# Install ROS
ENV ROS_DISTRO=melodic
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y ros-${ROS_DISTRO}-ros-base && \
    apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator && \
    apt-get install -y python-wstool python-catkin-pkg python-catkin-tools && \
    apt-get install -y ros-${ROS_DISTRO}-rviz ros-${ROS_DISTRO}-rqt && \
    rosdep init && rosdep fix-permissions && rosdep update -y

# Create catkin workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

# Install ROS packages
COPY install/i3dr-mapping-demo-http.rosinstall /tmp/i3dr-mapping-demo-http.rosinstall
ENV ROSINSTALL_PACKAGES_UPDATE_CACHE=tn4g0vgupRcxq9oIYNVj
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    wstool init src /tmp/i3dr-mapping-demo-http.rosinstall && \
    echo "yaml https://raw.githubusercontent.com/i3drobotics/pylon_camera/main/rosdep/pylon_sdk.yaml " > /etc/ros/rosdep/sources.list.d/15-plyon_camera.list && \
    apt update && rosdep update -y && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

# CUDA variables required JIT compilation
# CUDA_CACHE_PATH should be used to attach a volume for caching the JIT compilation
ENV CUDA_CACHE_MAXSIZE=2147483647
ENV CUDA_CACHE_DISABLE=0
ENV CUDA_CACHE_PATH=/root/.nv/ComputeCache

# Build catkin workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DWITH_I3DRSGM=ON

# Create folder for storing I3DRSGM license files
# Will copy *.lic files to correct folder/s on startup
RUN mkdir -p /root/.i3dr/lic

# Create package folder
RUN mkdir -p /root/catkin_ws/src/i3dr-ros-mapping-demo

# Add package file to package folder
COPY package.xml /root/catkin_ws/src/i3dr-ros-mapping-demo/
# Install ROS dependencies
RUN apt-get update && rosdep update -y && \
    rosdep install --from-paths src --ignore-src -y -r --rosdistro ${ROS_DISTRO}

# Add build files to package folder
COPY CMakeLists.txt /root/catkin_ws/src/i3dr-ros-mapping-demo/
# Build catkin workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DWITH_I3DRSGM=ON

# Copy source files to package folder
COPY config /root/catkin_ws/src/i3dr-ros-mapping-demo/config
COPY launch /root/catkin_ws/src/i3dr-ros-mapping-demo/launch

# Setup entrypoint
COPY docker/entry.bash /
RUN ["chmod", "+x", "/entry.bash"]
ENTRYPOINT ["/entry.bash"]