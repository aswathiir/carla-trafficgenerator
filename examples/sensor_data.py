import carla
import csv
import time
import random

# Connect to CARLA
client = carla.Client('localhost', 2000)
client.set_timeout(20.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()

# Get all spawned vehicles
vehicles = world.get_actors().filter('vehicle.*')
if not vehicles:
    print("No vehicles found! Run generate_traffic.py first.")
    exit()

print(f"Found {len(vehicles)} vehicles. Attaching sensors...")

# CSV File Setup
csv_file = "sensor_data.csv"
header = ["Timestamp", "Vehicle_ID", "Lidar Points", "Latitude", "Longitude", "Altitude", 
          "Accel X", "Accel Y", "Accel Z", "Gyro X", "Gyro Y", "Gyro Z", "Collision"]
with open(csv_file, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(header)

# Sensor storage
sensors = []
sensor_data = {}

# Attach Sensors to Each Vehicle
for vehicle in vehicles:
    vehicle_id = vehicle.id
    sensor_data[vehicle_id] = {
        "Lidar": 0, "GPS": (0, 0, 0), "IMU": (0, 0, 0, 0, 0, 0), "Collision": "No"
    }

    ### 📌 **LiDAR Sensor**
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('range', '50')  # 50m range
    lidar_transform = carla.Transform(carla.Location(x=0, z=2))  # Roof mount
    lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
    sensors.append(lidar)

    def process_lidar(point_cloud, vid=vehicle_id):
        sensor_data[vid]["Lidar"] = len(point_cloud.raw_data) // 4

    lidar.listen(lambda point_cloud: process_lidar(point_cloud))

    ### 📌 **GPS Sensor**
    gps_bp = blueprint_library.find('sensor.other.gnss')
    gps = world.spawn_actor(gps_bp, carla.Transform(), attach_to=vehicle)
    sensors.append(gps)

    def process_gps(data, vid=vehicle_id):
        sensor_data[vid]["GPS"] = (data.latitude, data.longitude, data.altitude)

    gps.listen(lambda data: process_gps(data))

    ### 📌 **IMU Sensor**
    imu_bp = blueprint_library.find('sensor.other.imu')
    imu = world.spawn_actor(imu_bp, carla.Transform(), attach_to=vehicle)
    sensors.append(imu)

    def process_imu(data, vid=vehicle_id):
        sensor_data[vid]["IMU"] = (data.accelerometer.x, data.accelerometer.y, data.accelerometer.z,
                                   data.gyroscope.x, data.gyroscope.y, data.gyroscope.z)

    imu.listen(lambda data: process_imu(data))

    ### 📌 **Collision Sensor**
    collision_bp = blueprint_library.find('sensor.other.collision')
    collision = world.spawn_actor(collision_bp, carla.Transform(), attach_to=vehicle)
    sensors.append(collision)

    def process_collision(event, vid=vehicle_id):
        print(f"Collision detected for Vehicle {vid} with {event.other_actor}")
        sensor_data[vid]["Collision"] = "Yes"

    collision.listen(lambda event: process_collision(event))

# Collect Data for 20 Seconds
print("Collecting sensor data...")
start_time = time.time()
try:
    while time.time() - start_time < 20:
        with open(csv_file, "a", newline="") as f:
            writer = csv.writer(f)
            for vid, data in sensor_data.items():
                writer.writerow([
                    time.time(), vid, data["Lidar"],
                    *data["GPS"], *data["IMU"], data["Collision"]
                ])
        time.sleep(0.5)  # Collect data every 0.5 seconds

finally:
    print("Stopping sensors and saving data...")
    for sensor in sensors:
        sensor.stop()
        sensor.destroy()
    print(f"Sensor data saved to {csv_file}.")
