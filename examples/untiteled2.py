import sys
import os
import csv
import time
import threading
import uuid
import numpy as np
import carla

sys.path.append(r"C:\Users\lazya\OneDrive\Documents\sem 4\ml\CARLA_Latest\WindowsNoEditor\PythonAPI\carla")
sys.path.append(r"C:\Users\lazya\OneDrive\Documents\sem 4\ml\CARLA_Latest\WindowsNoEditor\PythonAPI")
sys.path.append(r"C:\Users\lazya\scenario_runner")

# Connect to CARLA
client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()

# Spawn Multiple Vehicles
spawn_points = world.get_map().get_spawn_points()
vehicles = []
vehicle_data = {}
for i in range(min(30, len(spawn_points))):
    vehicle_bp = blueprint_library.filter("vehicle.*")[i % len(blueprint_library.filter("vehicle.*"))]
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[i])
    if vehicle:
        vehicle.set_autopilot(True)
        vehicle_id = str(uuid.uuid4())[:8]  # Generate unique vehicle ID as a string
        vehicle_data[vehicle_id] = {"vehicle": vehicle, "sensor_data": {}}
        vehicles.append(vehicle)

# CSV Setup
csv_file = "traffic_violations_with_sensors.csv"
csv_lock = threading.Lock()
if not os.path.exists(csv_file):
    with open(csv_file, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Vehicle_ID", "Violation_Type", "Details", 
                         "LiDAR Points", "Latitude", "Longitude", "Altitude", 
                         "Accel X", "Accel Y", "Accel Z", "Gyro X", "Gyro Y", "Gyro Z", "Collision"])

# Function to log violations
def log_violation(vehicle_id, violation_type, details):
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"\U0001F6A8 {timestamp} - Vehicle {vehicle_id}: {violation_type} - {details}")
    with csv_lock:
        with open(csv_file, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, vehicle_id, violation_type, details, 
                             vehicle_data[vehicle_id]["sensor_data"].get("LiDAR", 0),
                             *vehicle_data[vehicle_id]["sensor_data"].get("GPS", (0, 0, 0)),
                             *vehicle_data[vehicle_id]["sensor_data"].get("IMU", (0, 0, 0, 0, 0, 0)),
                             vehicle_data[vehicle_id]["sensor_data"].get("Collision", "No")])

# Attach Sensors to Vehicles
def attach_sensors(vehicle_id, vehicle):
    sensor_data = {"LiDAR": 0, "GPS": (0, 0, 0), "IMU": (0, 0, 0, 0, 0, 0), "Collision": "No"}
    blueprint_library = world.get_blueprint_library()
    
    # LiDAR
    lidar_bp = blueprint_library.find("sensor.lidar.ray_cast")
    lidar_bp.set_attribute("range", "50")
    lidar_sensor = world.spawn_actor(lidar_bp, carla.Transform(), attach_to=vehicle)
    lidar_sensor.listen(lambda data: sensor_data.update({"LiDAR": len(data.raw_data) // 4}))
    
    # GPS
    gps_bp = blueprint_library.find("sensor.other.gnss")
    gps_sensor = world.spawn_actor(gps_bp, carla.Transform(), attach_to=vehicle)
    gps_sensor.listen(lambda data: sensor_data.update({"GPS": (data.latitude, data.longitude, data.altitude)}))
    
    # IMU
    imu_bp = blueprint_library.find("sensor.other.imu")
    imu_sensor = world.spawn_actor(imu_bp, carla.Transform(), attach_to=vehicle)
    imu_sensor.listen(lambda data: sensor_data.update({"IMU": (data.accelerometer.x, data.accelerometer.y, data.accelerometer.z,
                                                           data.gyroscope.x, data.gyroscope.y, data.gyroscope.z)}))
    
    # Collision
    collision_bp = blueprint_library.find("sensor.other.collision")
    collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=vehicle)
    collision_sensor.listen(lambda event: log_violation(vehicle_id, "Collision", f"Collision with {event.other_actor.type_id}"))
    
    vehicle_data[vehicle_id]["sensor_data"] = sensor_data

for v_id, v_info in vehicle_data.items():
    attach_sensors(v_id, v_info["vehicle"])

# Traffic Violation Detection
def monitor_violations():
    while True:
        for v_id, v_info in vehicle_data.items():
            vehicle = v_info["vehicle"]
            sensor_data = v_info["sensor_data"]
            velocity = vehicle.get_velocity()
            speed = 3.6 * np.linalg.norm([velocity.x, velocity.y, velocity.z])
            if speed > 50:
                log_violation(v_id, "Speeding", f"Speed: {speed:.2f} km/h (Limit: 50 km/h)")
            traffic_light = vehicle.get_traffic_light()
            if traffic_light and traffic_light.state == carla.TrafficLightState.Red:
                log_violation(v_id, "Running Red Light", "Traffic light was red")
            waypoint = world.get_map().get_waypoint(vehicle.get_location(), project_to_road=True)
            if waypoint.lane_type != carla.LaneType.Driving:
                log_violation(v_id, "Lane Violation", "Driving outside lane boundaries")
            if sensor_data["LiDAR"] > 100:
                log_violation(v_id, "Illegal Overtaking", "Detected overtaking maneuver")
        time.sleep(1)

# Start Monitoring Thread
violation_thread = threading.Thread(target=monitor_violations)
violation_thread.daemon = True
violation_thread.start()

# Run Simulation
try:
    while True:
        time.sleep(5)
except KeyboardInterrupt:
    print("\nStopping the simulation...")
    for vehicle in vehicles:
        vehicle.destroy()
    print("All vehicles and sensors stopped.")
