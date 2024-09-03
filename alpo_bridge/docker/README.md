steps to build and alpo_bridge image

ssh-add ~/.ssh/id_rsa 
docker login gitlab-registry.irstea.fr
docker build -t gitlab-registry.irstea.fr/romea_ros2/interfaces/vehicles/alpo/alpo_bridge --ssh default=$SSH_AUTH_SOCK .
docker push -t gitlab-registry.irstea.fr/romea_ros2/interfaces/vehicles/alpo/alpo_bridge

launch alpo_bridge un using docker run
docker run -it --rm gitlab-registry.irstea.fr/romea_ros2/interfaces/vehicles/alpo/alpo_bridge ros2 run alpo_bridge alpo_bridge 
