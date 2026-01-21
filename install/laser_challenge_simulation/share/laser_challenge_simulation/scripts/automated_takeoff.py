#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import time

class AutomatedTakeoff(Node):
    def __init__(self):
        super().__init__('automated_takeoff')
        self.get_logger().info('Iniciando conexão com o drone...')
        
        # Conecta ao SITL (Porta padrão 14551 para scripts externos)
        self.vehicle = mavutil.mavlink_connection('udp:127.0.0.1:14551')
        
        self.vehicle.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, 
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        
        self.vehicle.wait_heartbeat()
        self.get_logger().info('Drone conectado!')
        
        self.arm_and_takeoff(5.0) # Altitude de 5 metros

    def arm_and_takeoff(self, target_altitude):
        # 1. Mudar para o modo GUIDED
        self.get_logger().info('Mudando para modo GUIDED...')
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            1, 4, 0, 0, 0, 0, 0) # 1 = Custom Mode, 4 = GUIDED em ArduCopter

        # 2. Armar Motores
        self.get_logger().info('Armando motores...')
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)

        # Aguarda confirmação do armamento
        time.sleep(2)

        # 3. Decolagem
        self.get_logger().info(f'Decolando para {target_altitude} metros...')
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, target_altitude)

def main(args=None):
    rclpy.init(args=args)
    node = AutomatedTakeoff()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()