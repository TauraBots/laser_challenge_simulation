#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
# Mudança para ros_gz_interfaces no Harmonic
from ros_gz_interfaces.srv import SpawnEntity
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
import os

# Lista das posições (mantida do seu código original)
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
        

        self.client = self.create_client(SpawnEntity, '/world/default/create')      

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço /spawn_entity (GZ Sim) não disponível, a aguardar...')

        self.declare_parameter('challenge_stage', 'stage_one')
        self.challenge_stage = self.get_parameter('challenge_stage').get_parameter_value().string_value

        self.run_spawner()

    def spawn(self, model_name, x, y, z, model_path):
        if not os.path.exists(model_path):
            self.get_logger().error(f"Ficheiro não encontrado: {model_path}")
            return

        with open(model_path, "r") as f:
            model_xml = f.read()

        request = SpawnEntity.Request()
        request.entity_factory.name = model_name
        request.entity_factory.sdf = model_xml
        request.entity_factory.pose.position.x = float(x)
        request.entity_factory.pose.position.y = float(y)
        request.entity_factory.pose.position.z = float(z)
        
        self.client.call_async(request)
        self.get_logger().info(f"Spawn enviado: {model_name} em ({x}, {y}, {z})")

    def run_spawner(self):
        try:
            pkg_path = get_package_share_directory('laser_challenge_simulation')
        except Exception:
            self.get_logger().error("Pacote não encontrado.")
            return

        # 1. Spawn do Drone Iris (O "Pulo do Gato" para a integração)
        iris_path = os.path.join(pkg_path, "models", "iris", "model.sdf")

        self.spawn("iris", 0.0, 0.0, 1.0, iris_path)


        # 2. Lógica de Estágios (Mantida do original)
        indices = random.sample(range(1, 38), 3)
        
        if self.challenge_stage == "stage_one":
            self.spawn_bases(indices, pkg_path)
            
        elif self.challenge_stage == "stage_three":
            self.spawn_bases(indices, pkg_path)
            indices.extend([39, 40])
            self.spawn_qr_boxes(indices, pkg_path)

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
    rclpy.spin_once(node, timeout_sec=5.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()