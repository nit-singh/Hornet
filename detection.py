import asyncio
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
from rplidar import RPLidar

# === CONFIG ===
LIDAR_PORT = '/dev/ttyUSB0'  # Replace with your RPLiDAR port
OBSTACLE_DETECTION_ANGLE_RANGE = (330, 30)  # Front-facing detection cone
OBSTACLE_DISTANCE_THRESHOLD = 1000  # In mm (1 meter)
DETOUR_OFFSET_Y = 5.0  # How much to sidestep in Y (east) direction

# === LIDAR OBSTACLE DETECTION ===
def check_obstacle(lidar):
    for scan in lidar.iter_scans(max_buf_meas=500):
        for (_, angle, distance) in scan:
            # Normalize angle to 0-360
            if angle < OBSTACLE_DETECTION_ANGLE_RANGE[1] or angle > OBSTACLE_DETECTION_ANGLE_RANGE[0]:
                if distance < OBSTACLE_DISTANCE_THRESHOLD and distance > 100:
                    print(f"[Obstacle] Detected at angle {angle:.1f}°, distance {distance} mm")
                    return True
        break  # Only process one scan
    return False

# === DRONE FLIGHT ===
async def go_to(drone, x, y, z, yaw=0.0):
    print(f"Flying to: x={x}, y={y}, z={z}")
    await drone.offboard.set_position_ned(PositionNedYaw(x, y, z, yaw))

def find_detour(current, goal):
    return (current[0], current[1] + DETOUR_OFFSET_Y, current[2])  # Sidestep right (east)

async def main():
    # Connect to drone
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    # Connect to RPLiDAR
    lidar = RPLidar(LIDAR_PORT)
    print("RPLiDAR initialized.")

    # Arm and takeoff
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(3.0)
    await drone.action.takeoff()
    await asyncio.sleep(5)

    await drone.offboard.start()

    # Define waypoints (in NED coordinates)
    start = (0.0, 0.0, -3.0)
    goal = (20.0, 0.0, -3.0)
    current = start

    await go_to(drone, *goal)

    while True:
        obstacle = check_obstacle(lidar)
        if obstacle:
            print("Obstacle detected! Hovering and planning detour.")
            await go_to(drone, *current)
            await asyncio.sleep(2)

            detour = find_detour(current, goal)
            print(f"Detouring to: {detour}")
            await go_to(drone, *detour)
            await asyncio.sleep(5)

            print("Heading back to original goal.")
            await go_to(drone, *goal)

        # Check distance to goal
        async for pos in drone.telemetry.position_velocity_ned():
            distance = math.sqrt((pos.position.north_m - goal[0])**2 + (pos.position.east_m - goal[1])**2)
            if distance < 1.0:
                print("Goal reached.")
                break
        break  # Exit once destination reached

    await drone.action.land()
    print("Landing...")

    # Cleanup
    lidar.stop()
    lidar.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
