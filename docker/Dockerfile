ARG UBUNTU_VERSION=20.04
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

RUN apt update && add-apt-repository ppa:linuxuprising/libpng12 -y && \
    apt install libpng12-0 && \
    rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

RUN wget https://www.baslerweb.com/fp-1615275617/media/downloads/software/pylon_software/pylon_6.2.0.21487-deb0_amd64.deb && \
    wget https://bugs.launchpad.net/~ubuntu-security-proposed/+archive/ubuntu/ppa/+build/18845128/+files/libicu55_55.1-7ubuntu0.5_amd64.deb && \
    wget http://security.ubuntu.com/ubuntu/pool/universe/x/xerces-c/libxerces-c3.1_3.1.3+debian-1_amd64.deb && \
    wget https://github.com/i3drobotics/phobosIntegration/releases/download/v1.0.54/Phobos-1.0.54-x86_64_reducedTemplates.deb && \
    dpkg -i pylon_6.2.0.21487-deb0_amd64.deb && \
    dpkg -i libicu55_55.1-7ubuntu0.5_amd64.deb && \
    dpkg -i libxerces-c3.1_3.1.3+debian-1_amd64.deb && \
    dpkg -i Phobos-1.0.54-x86_64_reducedTemplates.deb && \
    rm -rf pylon_6.2.0.21487-deb0_amd64.deb libicu55_55.1-7ubuntu0.5_amd64.deb libxerces-c3.1_3.1.3+debian-1_amd64.deb Phobos-1.0.54-x86_64_reducedTemplates.deb

# CUDA variables required JIT compilation
# CUDA_CACHE_PATH should be used to attach a volume for caching the JIT compilation
ENV CUDA_CACHE_MAXSIZE=2147483647
ENV CUDA_CACHE_DISABLE=0
ENV CUDA_CACHE_PATH=/root/.nv/ComputeCache

# # Create folder for storing I3DRSGM license files
# # Will copy *.lic files to correct folder/s on startup
# RUN mkdir -p /root/.i3dr/lic

# Install ROS
ENV ROS_DISTRO=noetic
ENV PYTHON_PREFIX=python3
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y ros-${ROS_DISTRO}-ros-base && \
    apt-get install -y ${PYTHON_PREFIX}-rosdep ${PYTHON_PREFIX}-rosinstall ${PYTHON_PREFIX}-rosinstall-generator && \
    apt-get install -y ${PYTHON_PREFIX}-wstool ${PYTHON_PREFIX}-catkin-pkg ${PYTHON_PREFIX}-catkin-tools && \
    apt-get install -y ros-${ROS_DISTRO}-rviz ros-${ROS_DISTRO}-rqt && \
    rosdep init && rosdep fix-permissions && rosdep update -y

# Install OpenCV dependencies
RUN apt update && \
    apt install -y \
    cmake gcc g++ git \
    ${PYTHON_PREFIX}-dev ${PYTHON_PREFIX}-numpy \
    libavcodec-dev libavformat-dev libswscale-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev \
    libgtk-3-dev \
    libpng-dev libjpeg-dev libopenexr-dev libtiff-dev libwebp-dev && \
    rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
# Build OpenCV from source
ENV OPENCV_BRANCH=3.4.18
RUN mkdir -p /root/opencv_build && \
    cd /root/opencv_build && \
    git clone https://github.com/opencv/opencv.git -b ${OPENCV_BRANCH} && \
    git clone https://github.com/opencv/opencv_contrib.git -b ${OPENCV_BRANCH} && \
    cd /root/opencv_build/opencv && \
    mkdir build
RUN cd /root/opencv_build/opencv/build && \
    cmake ../ \
    -DOPENCV_EXTRA_MODULES_PATH=/root/opencv_build/opencv_contrib/modules \
    -DWITH_CUDA=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_DOCS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_opencv_apps=OFF
RUN cd /root/opencv_build/opencv/build && \
    make -j$(nproc)
RUN cd /root/opencv_build/opencv/build && \
    make install && \
    cd /root/ && \
    # cleanup
    rm -rf /root/opencv_build

# Create catkin workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

# Install vision_opencv ROS package from source
RUN git clone https://github.com/ros-perception/vision_opencv.git /root/catkin_ws/src/vision_opencv && \
    git clone https://github.com/ros-perception/image_pipeline.git /root/catkin_ws/src/image_pipeline
# Install ROS dependencies
ENV ROS_VISIMG_UPDATE_CACHE=YSFQ39ks2X5QXhwOB5mT
RUN apt-get update && rosdep update -y && \
    rosdep install --from-paths src -y -r --rosdistro ${ROS_DISTRO} \
    --skip-keys="libopencv-dev python3-opencv cv_bridge image_geometry stereo_image_proc image_view image_rotate image_publisher image_proc image_pipeline depth_image_proc camera_calibration"
# Build catkin workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release

# Install ROS packages
COPY install/i3dr-mapping-demo-http.rosinstall /tmp/i3dr-mapping-demo-http.rosinstall
ENV ROSINSTALL_PACKAGES_UPDATE_CACHE=tn4g0vgupRcxq9oIYNVj
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    wstool init src /tmp/i3dr-mapping-demo-http.rosinstall && \
    echo "yaml https://raw.githubusercontent.com/i3drobotics/pylon_camera/main/rosdep/pylon_sdk.yaml " > /etc/ros/rosdep/sources.list.d/15-plyon_camera.list && \
    apt update && rosdep update -y && \
    rosdep install --from-paths src -y -r --rosdistro ${ROS_DISTRO} \
    --skip-keys="libopencv-dev python3-opencv cv_bridge image_geometry stereo_image_proc image_view image_rotate image_publisher image_proc image_pipeline depth_image_proc camera_calibration"
# Build catkin workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DWITH_I3DRSGM=ON

RUN mkdir -p /root/catkin_ws/src/i3dr-ros-mapping-demo

# Add package file to package folder
COPY package.xml /root/catkin_ws/src/i3dr-ros-mapping-demo/
# Install ROS dependencies
RUN apt-get update && rosdep update -y && \
    rosdep install --from-paths src -y -r --rosdistro ${ROS_DISTRO} \
    --skip-keys="libopencv-dev python3-opencv cv_bridge image_geometry stereo_image_proc image_view image_rotate image_publisher image_proc image_pipeline depth_image_proc camera_calibration"

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
