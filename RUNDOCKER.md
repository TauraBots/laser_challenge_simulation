# 🛸 Laser Challenge Simulation - ROS 2 Humble

Este repositório contém o ambiente de simulação para o desafio de robótica aérea, portado para **ROS 2 Humble**. O projeto utiliza **Docker** para garantir compatibilidade e aceleração de hardware (GPU) na **NVIDIA Jetson Nano**.

---

## 🛠️ Pré-requisitos (Host)

Antes de iniciar, certifique-se de que sua Jetson Nano possui o Docker e o NVIDIA Container Toolkit instalados:

```bash
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker