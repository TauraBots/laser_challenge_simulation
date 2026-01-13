# Utiliza a imagem base da NVIDIA otimizada para Jetson com ROS 2 Humble
FROM dustynv/ros:humble-desktop-l4t-r32.7.1

# Evita interações durante a instalação de pacotes
ENV DEBIAN_FRONTEND=noninteractive

# Configura o caminho dos modelos para o Gazebo encontrar as peças do desafio
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ros2_ws/install/laser_challenge_simulation/share/laser_challenge_simulation/models

# Atualiza e instala dependências de sistema e ferramentas de build do ROS 2
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-msgs \
    ros-humble-geometry-msgs \
    ros-humble-cv-bridge \
    ros-humble-ament-index-python \
    python3-opencv \
    libgazebo11-dev \
    && rm -rf /var/lib/apt/lists/*

# Configura o ambiente de trabalho (Workspace)
ENV ROS_WS=/home/ros2_ws
WORKDIR $ROS_WS

# Copia o seu pacote para dentro do contentor (opcional se usar volumes)
# COPY . src/laser_challenge_simulation

# Instala dependências do ROS usando rosdep
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && \
    rosdep install -y --from-paths src --ignore-src --rosdistro humble && \
    rm -rf /var/lib/apt/lists/*

# Compila o workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select laser_challenge_simulation

# Configura o ficheiro de entrada para carregar o ambiente ROS automaticamente
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source $ROS_WS/install/setup.bash" >> ~/.bashrc

# Define o comando padrão
CMD ["bash"]