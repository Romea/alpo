## Update the registry

The following command allows to re-build and push the new image to the gitlab registry.
It requires to add your private ssh key to an ssh-agent because the Dockerfile clones the repos
using git.

```
cd docker
eval $(ssh-agent)
ssh-add ~/.ssh/id_rsa  # replace by the correct filename
docker login gitlab-registry.irstea.fr
docker compose build --push bridge
```

## Usage

Here is an example of docker compose service:

```yaml
services:
  bridge:
    image: gitlab-registry.irstea.fr/romea_ros2/interfaces/vehicles/alpo/alpo_bridge
    network_mode: host
    environment:
      - ROS_IP=192.168.100.2
      - ROS_MASTER_URI=http://192.168.100.1:11311
      - ROS_DOMAIN_ID=1
    command: >-
      ros2 run alpo_bridge alpo_bridge
        --ros-args
        -r __ns:=/alpo/base
        -p override_ros1_master:=false
```
