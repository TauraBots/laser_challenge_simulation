import asyncio
from mavsdk import System

async def run():
    # O SITL do ArduPilot geralmente envia MAVLink para a porta 14550
    drone = System()
    print("Aguardando conexão com o drone na porta 14550...")
    await drone.connect(system_address="udp://:14550")

    # Verifica se o drone está conectado (Heartbeat)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Drone conectado!")
            break

    print("-- Aguardando o GPS ter uma posição fixa (Global Estimate)...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- GPS OK! Pronto para decolar.")
            break

    print("-- Armando os motores...")
    try:
        await drone.action.arm()
    except Exception as e:
        print(f"Erro ao armar: {e}")
        return

    print("-- Decolando...")
    await drone.action.set_takeoff_altitude(5.0)
    await drone.action.takeoff()

    await asyncio.sleep(10)

    print("-- Pousando...")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())