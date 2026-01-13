#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, Point, Quaternion
from ament_index_python.packages import get_package_share_directory
import os

# Lista das posições pré-definidas (mantida do original)
bases_positions = [
    (6.5, -0.5, 0.05), (6.5, -1.5, 0.05), (6.5, -2.5, 0.05), (6.5, -3.5, 0.05), (6.5, -4.5, 0.05), (6.5, -5.5, 0.05),
    (5.5, 0.5, 0.05), (5.5, -0.5, 0.05), (5.5, -1.5, 0.05), (5.5, -2.5, 0.05), (5.5, -3.5, 0.05), (5.5, -4.5, 0.05), (5.5, -5.5, 0.05),
    (4.5, 0.5, 0.05), (4.5, -0.5, 0.05), (4.5, -1.5, 0.05), (4.5, -2.5, 0.05), (4.5, -3.5, 0.05), (4.5, -4.5, 0.05), (4.5, -5.5, 0.05),
    (3.5, -1.5, 0.05), (3.5, -2.5, 0.05), (3.5, -3.5, 0.05), (3.5, -4.5, 0.05), (3.5, -5.5, 0.05), 
    (2.5, -1.5, 0.05), (2.5, -2.5, 0.05), (2.5, -3.5, 0.05), (2.5, -4.5, 0.05), (2.5, -5.5, 0.05),
    (1.5, -1.5, 0.05), (1.5, -2.5, 0.05), (1.5, -3.5, 0.05), (1.5, -4.5, 0.05), (1.5, -5.5, 0.05),
    (0.5, -2.5, 0.05), (0.5, -3.5, 0.05), (0.5, -4.5, 0.05), 
    (0.0, -6.0, 1.5), (2.75, 0.0, 1.0)
]

class BaseSpawner(Node):
    def __init__(self):
        super().__init__('base_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço /spawn_entity não disponível, a aguardar...')

        # Declaração de parâmetros (equivalente a rospy.get_param)
        self.declare_parameter('challenge_stage', 'stage_one')
        self.declare_parameter('bases_spawn', '')

        self.challenge_stage = self.get_parameter('challenge_stage').get_parameter_value().string_value
        bases_spawn_str = self.get_parameter('bases_spawn').get_parameter_value().string_value

        # Parsing da lista de bases
        self.bases_spawn_param = []
        if bases_spawn_str:
            clean_str = bases_spawn_str.strip('[]')
            self.bases_spawn_param = [int(idx.strip()) for idx in clean_str.split(",") if idx.strip()]

        # Lógica de execução baseada no estágio
        self.run_spawner()

    def spawn(self, model_name, x, y, z, model_path):
        self.get_logger().info(f"A preparar spawn do modelo: {model_name} em ({x}, {y}, {z})")
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"Ficheiro do modelo não encontrado: {model_path}")
            return

        with open(model_path, "r") as f:
            model_xml = f.read()

        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = model_xml
        request.initial_pose.position.x = float(x)
        request.initial_pose.position.y = float(y)
        request.initial_pose.position.z = float(z)
        
        future = self.client.call_async(request)
        # Em scripts simples, podemos esperar pela resposta de forma síncrona se necessário
        # rclpy.spin_until_future_complete(self, future)

    def run_spawner(self):
        # Obter o caminho base do pacote de forma dinâmica (evita caminhos fixos /home/wagner/...)
        try:
            pkg_path = get_package_share_directory('laser_challenge_simulation')
        except Exception:
            self.get_logger().error("Pacote laser_challenge_simulation não encontrado no index do ROS 2.")
            return

        if self.challenge_stage == "stage_one":
            if not self.bases_spawn_param:
                self.bases_spawn_param = random.sample(range(1, 38), 3)
            self.spawn_bases(self.bases_spawn_param, pkg_path)
            
        elif self.challenge_stage == "stage_three":
            if not self.bases_spawn_param:
                self.bases_spawn_param = random.sample(range(1, 38), 3)
            self.spawn_bases(self.bases_spawn_param, pkg_path)
            self.bases_spawn_param.extend([39, 40])
            self.spawn_qr_boxes(self.bases_spawn_param, pkg_path)

    def spawn_bases(self, indices, pkg_path):
        model_path = os.path.join(pkg_path, "models", "landing_platform", "model.sdf")
        for idx in indices:
            x, y, z = bases_positions[idx - 1]
            self.spawn(f"base_{idx}", x, y, z, model_path)

    def spawn_qr_boxes(self, indices, pkg_path):
        qr_codes = ['a', 'b', 'c', 'd', 'e']
        random.shuffle(qr_codes)
        for idx in indices:
            if not qr_codes: break
            code = qr_codes.pop()
            x, y, z = bases_positions[idx - 1]
            model_path = os.path.join(pkg_path, "models", f"qrcode_box_{code}", "model.sdf")
            self.spawn(f"qrcode_box_{code}", x, y, z + 0.15, model_path)

def main(args=None):
    rclpy.init(args=args)
    node = BaseSpawner()
    # Como o spawn é assíncrono, giramos um pouco para garantir que os pedidos saem
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()