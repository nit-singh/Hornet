#added rotation offset
#full updated code with MissionItem-based movement ===
# Part 1: Imports and Global Setup
import numpy as np
import threading
import time
import math
from rplidar import RPLidar
import matplotlib.pyplot as plt
import asyncio
from mavsdk import System
from mavsdk.action import Action
from mavsdk.offboard import Offboard, VelocityNedYaw, PositionNedYaw
from mavsdk.telemetry import Telemetry
from mavsdk.mission import Mission, MissionItem, MissionPlan, MissionProgress
from mavsdk.param import Param
from pymavlink import mavutil

# Constants
BOARD_SIZE = 502 # 2.5 metre each side of drone
CENTER = BOARD_SIZE / 2
SECTOR_COUNT = 360 #updated to 1 degree per sector  
C = 100
K = 5  
V = 10 #constant magnitude velocity given for instructions
Hist_threshold = 100 #Threshold to determine if its safe to move in the sector or not
#Can put a dynamic threshold which is interdependent on velocity of drone
alt_change_sectors = 30 # if width of safe sectors less than this , we change altitude
critical_d = 75  #Critical protocol will be implemented if any detection less than this
#also later include a lower bound as to ignore drone parts if lidar gets tilted
min_critical_gap = 90 # min gap in degrees needed in critical protocol to steer otherwise altitude change
# Global variables to store previous GPS position
prev_lat, prev_lon,prev_alt_interval = None, None,None
# LiDAR device path
PORT_NAME = "/dev/tty.usbserial-0001"  # Update with actual port

# Global variables for threading
latest_scan = []
scan_lock = threading.Lock()
running = True  # Flag to stop the LiDAR properly
drone = None
# Initialize board
board = np.zeros((BOARD_SIZE, BOARD_SIZE), dtype=float)

# === New Sub-Mission Functions ===

def offset_gps(lat, lon, dn, de):
    dlat = dn / 111111
    dlon = de / (111111 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon
async def create_nudge_mission_item(drone, dn=0, de=0, dalt=0.3):
    async for pos in drone.telemetry.position():
        lat = pos.latitude_deg
        lon = pos.longitude_deg
        alt = pos.relative_altitude_m + dalt
        new_lat, new_lon = offset_gps(lat, lon, dn, de)
        return MissionItem(
            new_lat, new_lon, alt,
            speed=0.5,
            is_fly_through=True,
            gimbal_pitch_deg=0,
            gimbal_yaw_deg=0,
            camera_action=CameraAction.NONE
        )
async def execute_sub_mission(drone, mission_item):
    await drone.mission.pause_mission()
    await drone.mission.clear_mission()
    await drone.mission.upload_mission(MissionPlan([mission_item]))
    await drone.mission.start_mission()

    while True:
        async for progress in drone.mission.mission_progress():
            if progress.current == progress.total:
                return
        await asyncio.sleep(0.5)

# === Rest of Your Original Logic ===
# (collect_lidar_data, update_board_by_gps, gps_to_local, shift_board, update_certainity_grid,
#  update_polar_histogram, find_force_vector, find_critical_angle,
#  plot_histogram, safest_angle)
async def collect_lidar_data():
    global latest_scan, running
    global drone
    lidar = RPLidar(PORT_NAME, baudrate=256000)
    try:
        temp_scan = []
        d = 2 * CENTER * 10
        last_angle = 0
        async for attitude in drone.telemetry.attitude_euler():
             yaw_deg = attitude.yaw_deg
             break
        
        offset = 90 - yaw_deg
        for scan in lidar.iter_scans(max_buf_meas=10000):
            for (strength, angle, distance) in scan:
                if angle > last_angle + 2:
                    i = last_angle + 1
                    while i < angle:
                        temp_scan.append((strength, i + offset, d))
                        i += 0.5
                temp_scan.append((strength, angle + offset, distance))
                last_angle = angle
            with scan_lock:
                latest_scan = temp_scan.copy()
            temp_scan = []
            if not running:
                break
    except Exception as e:
        print(f"LiDAR Error: {e}")
    finally:
        lidar.stop_motor()
        lidar.stop()
        lidar.disconnect()
async def update_board_by_gps(drone):
    global prev_lat, prev_lon,prev_alt_interval
    global board
    # Get current GPS position
    telemetry = await drone.telemetry.position()
    lat_curr, lon_curr = telemetry.latitude_deg, telemetry.longitude_deg
    # Get current altitude
    telemetry = await drone.telemetry.position()
    current_alt = telemetry.relative_altitude_m  # Altitude in meters
    # Calculate the current interval
    current_alt_interval = int(current_alt / 0.3)
    # Check if the interval has changed
    if current_alt_interval != prev_alt_interval:

        print(f"Drone moved to a new altitude interval: {current_alt_interval * 0.3}m")
        board.fill(0)
        prev_alt_interval = current_alt_interval
        return board
    prev_alt_interval = current_alt_interval
    # Calculate displacement in meters
    dx, dy = gps_to_local(lat_curr, lon_curr, prev_lat, prev_lon)
    # Update previous position
    prev_lat, prev_lon = lat_curr, lon_curr
    # Shift board in the opposite direction of displacement
    return shift_board(board, dx, dy)
def gps_to_local(lat_curr, lon_curr, lat_prev, lon_prev):
    """Convert GPS displacement to local X, Y displacement in meters"""
    dy = mavutil.meters_per_deg_lat(lat_curr) * (lat_curr - lat_prev)
    dx = mavutil.meters_per_deg_lon(lat_curr) * (lon_curr - lon_prev)
    return int(round(dx)), int(round(dy))
def shift_board(board, dx, dy):
    """
    Shifts the board values in the opposite direction of the drone's displacement.
    The displacement vector (dx, dy) is given in grid cell units.
    For example, if the displacement is (dx=+2, dy=+1) [drone moved right 2 cells and up 1 cell],
    then we shift the board values by (-2, -1), so that the obstacles "move" left 2 cells and down 1 cell.
    The function returns a new board with the values shifted.
    """
    rows, cols = board.shape  # Assume board is a NumPy array
    # Compute shift amounts (opposite of displacement)
    shift_x = -int(round(dx))
    shift_y = -int(round(dy))
    # Create a new board filled with zeros
    new_board = np.zeros_like(board)
    # Loop through each cell and shift it if the new indices are within bounds
    for x in range(rows):

        for y in range(cols):

            new_x = x + shift_x

            new_y = y + shift_y

            if 0 <= new_x < rows and 0 <= new_y < cols:

                new_board[new_x, new_y] = board[x, y]
    return new_board
# Certainty Grid Update Function
def update_certainity_grid(lidar_data):
    """Updates the certainty grid based on LiDAR readings."""
    global board
    #board.fill(0)  # Reset board
    critical_angles = []
    for _, angle, distance in lidar_data:
        with open("lidar_data.txt", "a") as file:
            file.write(f"({angle},{distance})\n")

        theta = math.radians(angle)
        distance = distance/10 #converting mm to cm
        if distance<=critical_d:
            critical_angles.append(angle)
        x = int(CENTER + distance * math.cos(theta))
        y = int(CENTER + distance * math.sin(theta))

        if 0 <= x < BOARD_SIZE and 0 <= y < BOARD_SIZE:
            
            board[x,y] += 2  
            if board[x,y]>250:
                 board[x,y]=250 # !! CREATING THIS CAP IS SOMEHOW MAKING THE OUTPUT FLICKER (but at 500 flicker reduced)

        for i in range(1, int(distance)):
            xi = int(CENTER + i * math.cos(theta))
            yi = int(CENTER + i * math.sin(theta))
            if 0 <= xi < BOARD_SIZE and 0 <= yi < BOARD_SIZE:
                # board[xi, yi] *= (i / distance)  
                board[xi,yi]=0 #trying to reduce obstacle certainity at closer points
    
    return critical_angles
# Polar Histogram Update Function
def update_polar_histogram():
    """Computes and smooths the 1D polar histogram."""
    histogram = np.zeros(SECTOR_COUNT, dtype=float)

    for i in range(BOARD_SIZE):
        for j in range(BOARD_SIZE):
            if board[i, j] > 0:
                x = i - CENTER
                y = j - CENTER
                distance = math.sqrt(x**2 + y**2)
                sector = int((math.degrees(math.atan2(y, x)) + 360) % 360 / (360 / SECTOR_COUNT))

                if distance > 0:
                    #different variations to control dominance
                    # histogram[sector] += C * (board[i, j] ** 2) * (max(0, CENTER - distance))  
                    # histogram[sector] += C * (board[i, j] ** 2)/distance 
                    histogram[sector] += C * board[i, j]/(distance**2) #motivated by nature 


    smoothed_histogram = np.zeros(SECTOR_COUNT, dtype=float)
    for i in range(SECTOR_COUNT):
        total = 0
        weight_sum = 0
        for offset in range(-K, K + 1):
            index = (i + offset) % SECTOR_COUNT  
            weight = K + 1 - abs(offset)
            total += histogram[index] * weight
            weight_sum += weight
        smoothed_histogram[i] = total / weight_sum if weight_sum > 0 else 0

    return smoothed_histogram
# Force Vector Calculation Function
def find_force_vector(histogram):
    """Finds the net force vector based on the polar histogram."""
    net_x, net_y = 0, 0
    for i in range(SECTOR_COUNT):
        angle = math.radians(180 + (i * 360 / SECTOR_COUNT))
        magnitude = histogram[i]
        net_x += magnitude * math.cos(angle)
        net_y += magnitude * math.sin(angle)

    net_angle = math.degrees(math.atan2(net_y, net_x)) % 360
    net_magnitude = 0.01 * math.sqrt(net_x**2 + net_y**2)
    return net_angle, net_magnitude
def find_critical_angle(angles):
    #finding the widest gap
    # Sort the angles
    angles = sorted(angles)
    n = len(angles)
    if n < 2:
        return None, None  # Not enough points to form a gap
    
    # Find the largest gap considering circular behavior
    max_gap = 0
    max_midpoint = None

    for i in range(n):
        next_index = (i + 1) % n  # Wrap around for circularity
        gap = (angles[next_index] - angles[i]) % 360  # Ensure positive gap
        midpoint = (angles[i] + gap / 2) % 360  # Midpoint within 0-360 range

        if gap > max_gap:
            max_gap = gap
            max_midpoint = midpoint

    if max_gap<min_critical_gap:
        max_gap=-1
    return max_midpoint, max_gap
# Histogram Plotting Function
def plot_histogram(histogram, angle, magnitude):
    plt.figure(1)
    plt.clf()  
    x = np.arange(len(histogram))
    plt.bar(x * 2, histogram)
    plt.bar(int(angle), magnitude, color='green')

    plt.xlabel("Angle")
    plt.ylabel("Certainty of Obstacle")
    plt.title("Smoothed Histogram")
    plt.pause(0.1)  

    plt.figure(2)
    plt.clf()
    
    ax = plt.gca()
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_frame_on(False)  
    
    angle_rad = np.radians(angle)  
    x_end = magnitude * np.cos(angle_rad)
    y_end = magnitude * np.sin(angle_rad)

    segments = [[0, 0, magnitude, angle_rad], [x_end, y_end, magnitude / 10, np.radians((angle + 205) % 360)], [x_end, y_end, magnitude / 10, np.radians((angle + 155) % 360)]]
    
    for x_start, y_start, length, ang in segments:
        x_end = x_start + length * np.cos(ang)
        y_end = y_start + length * np.sin(ang)
        plt.plot([x_start, x_end], [y_start, y_end], 'r-', linewidth=2)  

    plt.pause(0.1)  
def safest_angle(arr):
    Hist_threshold= 0.2 * max(arr) # implementing new logic for threshold to make binary histogram
    #all values below 20% of max will be considered safe

    binary_mask = (np.array(arr) > Hist_threshold).astype(int) #Masking the histogram based on threshold
    n = len(binary_mask)
    #finding the longest chain of zeros (as no way point deteced)
    extended_mask = np.concatenate((binary_mask, binary_mask))  # Circular extension
    max_length, max_start = 0, 0
    current_length, current_start = 0, -1

    for i in range(2 * n):
        if extended_mask[i] == 0:
            if current_length == 0:
                current_start = i  # Start of a new zero sequence
            current_length += 1
        else:
            if current_length > max_length:
                max_length, max_start = current_length, current_start
            current_length = 0  # Reset for next sequence

    if current_length > max_length:
        max_length, max_start = current_length, current_start
    
    midpoint = (max_start + max_length // 2) % n

    if max_length<alt_change_sectors:
        midpoint = -1 #need to change altitude
    
    return midpoint*(360/SECTOR_COUNT)

# === Main Processing Function ===

async def process_lidar_data(drone):
    global running, board

    while running:
        board = await update_board_by_gps(drone)

        with scan_lock:
            scan_data = latest_scan.copy()

        if scan_data:
            print(f"Processing {len(scan_data)} points")
            critical_angles = update_certainity_grid(scan_data)
            histogram = update_polar_histogram()

            if not critical_angles:
                print("Critical Protocol Activated.")
                critic_direction, critic_gap = find_critical_angle(critical_angles)
                if critic_gap < 0:
                    print("Altitude Change at critical stage.")
                    item = await create_nudge_mission_item(drone, dalt=0.3)
                    await execute_sub_mission(drone, item)
                else:
                    print(f"Safe direction to steer: {critic_direction:.2f}")
                    dn = math.cos(math.radians(critic_direction)) * 1.0
                    de = math.sin(math.radians(critic_direction)) * 1.0
                    item = await create_nudge_mission_item(drone, dn=dn, de=de, dalt=0)
                    await execute_sub_mission(drone, item)
            else:
                direction, magnitude = find_force_vector(histogram)
                dir2 = safest_angle(histogram)
                if dir2 < 0:
                    print("Altitude Change!")
                    item = await create_nudge_mission_item(drone, dalt=0.3)
                    await execute_sub_mission(drone, item)
                else:
                    print(f"Repulsive: {direction:.2f}| Final: {dir2:.2f}| Mag: {magnitude:.2f}")
                    plot_histogram(histogram, direction, magnitude)
                    plot_histogram((np.array(histogram) > Hist_threshold).astype(int), abs(dir2), magnitude)
                    dn = math.cos(math.radians(dir2)) * 1.0
                    de = math.sin(math.radians(dir2)) * 1.0
                    item = await create_nudge_mission_item(drone, dn=dn, de=de, dalt=0)
                    await execute_sub_mission(drone, item)

        await asyncio.sleep(0.1)

# === Entry Point and LiDAR Thread ===

async def main():
    global drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(" Drone connected.")
            break

    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(5)

    lidar_thread = threading.Thread(target=collect_lidar_data)
    lidar_thread.start()

    try:
        await process_lidar_data(drone)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        global running
        running = False
        lidar_thread.join()
        plt.ioff()

if __name__ == "__main__":
    asyncio.run(main())
