# I3DR Mapping Demo ROS Package
Demonstration of mapping with I3DR stereo cameras in ROS.

## Build
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
git clone https://github.com/i3drobotics/i3dr-mapping-demo-ros.git ~/catkin_ws/src/i3dr-mapping-demo-ros
sudo echo "yaml https://raw.githubusercontent.com/i3drobotics/pylon_camera/main/rosdep/pylon_sdk.yaml " > /etc/ros/rosdep/sources.list.d/15-plyon_camera.list
wstool init src ~/catkin_ws/src/i3dr-mapping-demo-ros/i3dr-mapping-demo-http.rosinstall
sudo apt-get update
rosdep update -y
rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DWITH_I3DRSGM=ON
```

## Run
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch --wait i3dr_mapping_demo mapping_demo.launch camera_type:=titania camera_serial:=746974616e24318 stereo_algorithm:=2 rviz:=true exposure:=10000
```

## Docker
### Build
Docker configuration is available for running containerised. Use the following command to build the docker image:
```bash
docker compose -f docker/docker-compose.yml build
```
### Run
To use I3DRSGM for stereo matching in the container a valid license file must be placed in the `/docker/licenses` directory.  
A `.env` file must be created in the `docker` folder and configured with your `I3DRSGM_HOSTNAME` and `I3DRSGM_HOST_ID`. See the `.env.template` file for more information. This also has a variable for `DISPLAY_IP` which is used to forward display GUI if using Xserver for graphics output.
Run the docker configuration using the following command:
```bash
docker compose -f docker/docker-compose.yml up
```
