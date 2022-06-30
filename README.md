# I3DR Mapping Demo ROS Package
Demonstration of mapping with I3DR stereo cameras in ROS.

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
