#!/bin/bash

# Nome da imagem que definimos no Dockerfile
IMAGE_NAME="laser_challenge_ros2"
CONTAINER_NAME="laser_sim_container"

# 1. Permite que o Docker acesse o servidor X11 (Interface Gráfica)
echo "Configurando permissões de Display..."
xhost +local:docker > /dev/null

# 2. Verifica se já existe um container rodando com esse nome e o remove
if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "Removendo container antigo..."
    docker rm -f $CONTAINER_NAME > /dev/null
fi

# 3. Executa o container com as configurações necessárias para a Jetson
echo "Iniciando o ambiente ROS 2 Humble na Jetson Nano..."

docker run -it \
    --name $CONTAINER_NAME \
    --runtime nvidia \
    --network host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd):/home/ros2_ws/src/laser_challenge_simulation \
    $IMAGE_NAME