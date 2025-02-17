#!/usr/bin/env python3

import os
import random
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, Point, Quaternion

class BaseSpawner(Node):
    def __init__(self):
        super().__init__('base_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.declare_parameter('challenge_stage', 'stage_one')
        self.declare_parameter('bases_spawn', '')

        self.home_dir = os.path.expanduser("~")

    def spawn(self, model_name, x, y, z, model_path):
        self.get_logger().info(f"Preparing to spawn model: {model_name} at ({x}, {y}, {z})")
        pose = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        try:
            with open(model_path, "r") as model_file:
                model_xml = model_file.read()
        except FileNotFoundError:
            self.get_logger().error(f"Model file not found: {model_path}")
            return

        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = model_xml
        request.initial_pose = pose
        request.robot_namespace = ""
        request.reference_frame = "world"

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Spawned {model_name} at position ({x}, {y}, {z})")
        else:
            self.get_logger().error(f"Failed to spawn model {model_name}: {future.exception()}")

    def generate_random_positions(self, num_bases, x_limits, y_limits, z_limits, min_distance):
        positions = []
        while len(positions) < num_bases:
            x = random.uniform(x_limits[0], x_limits[1])
            y = random.uniform(y_limits[0], y_limits[1])
            z = random.uniform(z_limits[0], z_limits[1])  # Gera um valor aleatório para Z
            collision = False
            for pos in positions:
                # Verifica a distância em 3D (X, Y, Z)
                distance = ((x - pos[0])**2 + (y - pos[1])**2 + (z - pos[2])**2)**0.5
                if distance < min_distance:
                    collision = True
                    break
            if not collision:
                positions.append((x, y, z))  # Adiciona a posição com Z aleatório
        return positions

    def spawn_bases(self, num_bases, x_limits, y_limits, z_limits, min_distance):
        positions = self.generate_random_positions(num_bases, x_limits, y_limits, z_limits, min_distance)
        for idx, (x, y, z) in enumerate(positions):
            model_name = f"base_{idx}"
            self.spawn(model_name, x, y, z, self.home_dir + "/laser_uav_system_ws/src/laser_challenge_simulation/models/landing_platform/model.sdf")

def main(args=None):
    rclpy.init(args=args)
    base_spawner = BaseSpawner()
    
    home_dir = os.path.expanduser("~")
    challenge_stage = base_spawner.get_parameter('challenge_stage').get_parameter_value().string_value

    if challenge_stage == "stage_one":
        base_spawner.get_logger().info("Spawning cluttered environment...")
        cluttered_env_name = "cluttered_environment"
        cluttered_env_position = (-0.75, -1.0, 0.0)  # Posição do cluttered_environment
        base_spawner.spawn(cluttered_env_name, cluttered_env_position[0], cluttered_env_position[1], cluttered_env_position[2], home_dir + "/laser_uav_system_ws/src/laser_challenge_simulation/models/cluttered_environment/model.sdf")

        # Limites para as 5 bases normais
        x_limits_normal = (1.75, 6.25)  # x entre 0.5 e 7.5
        y_limits_normal = (0, -6)  # y entre 0.5 e 6.0
        z_limits_normal = (0.0, 1.5)  # z aleatório entre 0.0 e 1.5

        # Limites para a base especial
        x_limits_special = (0.25, 0.75)  # x entre 0.5 e 6.0
        y_limits_special = (-1.5, -6)  # y entre 6.0 e 7.5
        z_fixed_special = 1.52          # z fixo em 1.5

        # Distância mínima entre as bases (1.0 metro para evitar colisões)
        min_distance = 2.0

        # Spawn das 5 bases normais
        base_spawner.get_logger().info("Spawning 5 bases normais...")
        positions_normal = base_spawner.generate_random_positions(
            5, x_limits_normal, y_limits_normal, z_limits_normal, min_distance
        )
        for idx, (x, y, z) in enumerate(positions_normal):
            model_name = f"base_normal_{idx}"
            base_spawner.spawn(model_name, x, y, z, home_dir + "/laser_uav_system_ws/src/laser_challenge_simulation/models/landing_platform/model.sdf")

        # Spawn da base especial
        base_spawner.get_logger().info("Spawning 1 base especial...")
        x_special = random.uniform(x_limits_special[0], x_limits_special[1])
        y_special = random.uniform(y_limits_special[0], y_limits_special[1])
        z_special = z_fixed_special

        # Verifica se a base especial colide com as bases normais
        collision = False
        for pos in positions_normal:
            distance = ((x_special - pos[0])**2 + (y_special - pos[1])**2 + (z_special - pos[2])**2)**0.5
            if distance < min_distance:
                collision = True
                break

        if not collision:
            model_name = "base_especial"
            base_spawner.spawn(model_name, x_special, y_special, z_special, home_dir + "/laser_uav_system_ws/src/laser_challenge_simulation/models/landing_platform/model.sdf")
        else:
            base_spawner.get_logger().warn("Não foi possível spawnar a base especial sem colisão. Tente novamente.")
    
    elif challenge_stage == "stage_two":
        base_spawner.get_logger().info("Spawning cluttered environment...")
        cluttered_env_name = "cluttered_environment"
        cluttered_env_position = (-0.75, -1.0, 0.0)  # Posição do cluttered_environment
        base_spawner.spawn(cluttered_env_name, cluttered_env_position[0], cluttered_env_position[1], cluttered_env_position[2], home_dir + "/laser_uav_system_ws/src/laser_challenge_simulation/models/cluttered_environment/model.sdf")

        # Limites para as 3 bases aleatórias
        x_limits_random = (1.75, 6.25)  # x entre 0.5 e 7.5
        y_limits_random = (0, -6)  # y entre -0.5 e -7.5
        z_limits_random = (0.0, 1.5)  # z aleatório entre 0.0 e 1.5

        # Posições fixas para as 3 bases especiais
        fixed_positions = [
            (0.25, -2.5, 1.52),  # Base fixa 1
            (0.25, -4.0, 1.52),  # Base fixa 2
            (0.25, -5.5, 1.52)   # Base fixa 3
        ]

        # Distância mínima entre as bases (1.0 metro para evitar colisões em X-Y)
        min_distance = 1.0

        # Spawn das 3 bases aleatórias
        base_spawner.get_logger().info("Spawning 3 bases aleatórias...")
        positions_random = base_spawner.generate_random_positions(
            3, x_limits_random, y_limits_random, z_limits_random, min_distance
        )
        for idx, (x, y, z) in enumerate(positions_random):
            model_name = f"base_random_{idx}"
            base_spawner.spawn(model_name, x, y, z, home_dir + "/laser_uav_system_ws/src/laser_challenge_simulation/models/landing_platform/model.sdf")

        # Spawn das 3 bases fixas
        base_spawner.get_logger().info("Spawning 3 bases fixas...")
        for idx, (x, y, z) in enumerate(fixed_positions):
            model_name = f"base_fixa_{idx}"
            base_spawner.spawn(model_name, x, y, z, home_dir + "/laser_uav_system_ws/src/laser_challenge_simulation/models/landing_platform/model.sdf")

    elif challenge_stage == "stage_three": 
        base_spawner.get_logger().info("Spawning cluttered environment...")
        cluttered_env_name = "cluttered_environment"
        cluttered_env_position = (-0.75, -1.0, 0.0)  # Posição do cluttered_environment
        base_spawner.spawn(cluttered_env_name, cluttered_env_position[0], cluttered_env_position[1], cluttered_env_position[2], home_dir + "/laser_uav_system_ws/src/laser_challenge_simulation/models/cluttered_environment/model.sdf")
    elif challenge_stage == "stage_four":
        # Posição fixa para a base da fase 4
        fixed_position = (3.0, -3.25, 0.0)  # Base fixa

        # Spawn da base fixa
        base_spawner.get_logger().info("Spawning 1 base fixa...")
        model_name = "base_fixa_fase4"
        base_spawner.spawn(model_name, fixed_position[0], fixed_position[1], fixed_position[2],
                           home_dir + "/laser_uav_system_ws/src/laser_challenge_simulation/models/landing_platform/model.sdf")
        
        base_spawner.get_logger().info("Spawning cluttered environment...")
        cluttered_env_name = "cluttered_environment"
        cluttered_env_position = (-0.75, -1.0, 0.0)  # Posição do cluttered_environment
        base_spawner.spawn(cluttered_env_name, cluttered_env_position[0], cluttered_env_position[1], cluttered_env_position[2], home_dir + "/laser_uav_system_ws/src/laser_challenge_simulation/models/cluttered_environment/model.sdf")
    
    rclpy.spin(base_spawner)
    base_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
