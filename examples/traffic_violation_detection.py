import sys
import os
import csv
import time
import threading
import uuid
import numpy as np
import carla
from collections import defaultdict
import firebase_admin
from firebase_admin import credentials, db
from random import random

# Connect to CARLA
client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()

# Constants
MAX_SPEED_KPH = 50  # Speed limit for violations
MIN_VIOLATION_INTERVAL = 5  # Seconds between logging same violation type
DATA_COLLECTION_INTERVAL = 0.5  # Seconds between data collection
VIOLATION_RATIO = 0.5  # Target ratio of violation to non-violation samples
LIDAR_PROXIMITY_THRESHOLD = 1000  # Points threshold for proximity alert

class TrafficDataCollector:
    def __init__(self):
        self.vehicles = []
        self.vehicle_data = {}
        self.last_violation_time = defaultdict(dict)
        self.last_data_collection_time = defaultdict(float)
        self.violation_counts = defaultdict(int)
        self.non_violation_counts = defaultdict(int)
        
        # Add Firebase initialization
        self.firebase_initialized = False
        self._init_firebase()
        
        # CSV setup
        self.csv_file = "traffic_data_improved.csv"
        self.csv_lock = threading.Lock()
        self._init_csv()
        
        # Enable synchronous mode for precise timing
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        
    def _init_csv(self):
        """Initialize CSV file with headers"""
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, "w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([
                    "Event_ID", "Timestamp", "Vehicle_ID", "Is_Violation",
                    "Speed_KPH", "Throttle", "Brake", "Steer", "Heading",
                    "Latitude", "Longitude", "Altitude", 
                    "Accel_X", "Accel_Y", "Accel_Z", 
                    "Gyro_X", "Gyro_Y", "Gyro_Z",
                    "LiDAR_Points", "Proximity_Alert", "Collision_Partner",
                    "Traffic_Light_State", "Lane_Type"
                ])
                
    def _init_firebase(self):
        """Initialize Firebase connection"""
        try:
            cred = credentials.Certificate("serviceAccountKey.json")
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'https://carla-4f285-default-rtdb.firebaseio.com/'
            })
            self.firebase_initialized = True
            print("Firebase initialized successfully")
            
            # Test Firebase connection
            self.test_firebase()
        except Exception as e:
            print(f"Failed to initialize Firebase: {str(e)}")
            self.firebase_initialized = False
            
    def test_firebase(self):
        """Test Firebase connection"""
        if self.firebase_initialized:
            try:
                ref = db.reference('connection_test')
                ref.set({
                    'timestamp': time.strftime("%Y-%m-%d %H:%M:%S"),
                    'status': 'success',
                    'message': 'Firebase connection established'
                })
                print("Firebase test write successful")
            except Exception as e:
                print(f"Firebase test failed: {str(e)}")
            
    def spawn_vehicles(self, count=30):
        """Spawn vehicles with autopilot"""
        spawn_points = world.get_map().get_spawn_points()
        
        for i in range(min(count, len(spawn_points))):
            vehicle_bp = blueprint_library.filter("vehicle.*")[i % len(blueprint_library.filter("vehicle.*"))]
            try:
                vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[i])
                if vehicle:
                    vehicle.set_autopilot(True)
                    vehicle_id = str(uuid.uuid4())[:8]
                    self.vehicle_data[vehicle_id] = {
                        "vehicle": vehicle,
                        "sensors": {},
                        "last_state": {},
                        "violation_flags": set(),
                        "violation_count": 0,
                        "non_violation_count": 0
                    }
                    self.vehicles.append(vehicle)
                    self.attach_sensors(vehicle_id, vehicle)
            except Exception as e:
                print(f"Error spawning vehicle {i}: {str(e)}")
        
        print(f"Spawned {len(self.vehicles)} vehicles")

    def attach_sensors(self, vehicle_id, vehicle):
        """Attach sensors to vehicle with proper configuration"""
        try:
            # LiDAR with more realistic configuration
            lidar_bp = blueprint_library.find("sensor.lidar.ray_cast")
            lidar_bp.set_attribute("range", "50")  # 50 meters
            lidar_bp.set_attribute("rotation_frequency", "10")  # 10 Hz
            lidar_bp.set_attribute("channels", "32")  # 32 channels
            lidar_bp.set_attribute("points_per_second", "56000")  # 56,000 points/sec
            lidar_transform = carla.Transform(carla.Location(z=2.5))
            lidar_sensor = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
            
            # GPS
            gps_bp = blueprint_library.find("sensor.other.gnss")
            gps_sensor = world.spawn_actor(gps_bp, carla.Transform(), attach_to=vehicle)
            
            # IMU
            imu_bp = blueprint_library.find("sensor.other.imu")
            imu_sensor = world.spawn_actor(imu_bp, carla.Transform(), attach_to=vehicle)
            
            # Collision sensor
            collision_bp = blueprint_library.find("sensor.other.collision")
            collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=vehicle)
            
            # Lane invasion sensor
            lane_bp = blueprint_library.find("sensor.other.lane_invasion")
            lane_sensor = world.spawn_actor(lane_bp, carla.Transform(), attach_to=vehicle)
            
            # Store sensors and setup callbacks
            self.vehicle_data[vehicle_id]["sensors"] = {
                "lidar": lidar_sensor,
                "gps": gps_sensor,
                "imu": imu_sensor,
                "collision": collision_sensor,
                "lane": lane_sensor
            }
            
            # Setup callbacks
            lidar_sensor.listen(lambda data: self._update_lidar_data(vehicle_id, data))
            gps_sensor.listen(lambda data: self._update_gps_data(vehicle_id, data))
            imu_sensor.listen(lambda data: self._update_imu_data(vehicle_id, data))
            collision_sensor.listen(lambda event: self._handle_collision(vehicle_id, event))
            lane_sensor.listen(lambda event: self._handle_lane_invasion(vehicle_id, event))
            
        except Exception as e:
            print(f"Error attaching sensors to vehicle {vehicle_id}: {str(e)}")

    def _update_lidar_data(self, vehicle_id, data):
        """Process LiDAR data with more realistic point counts"""
        points = len(data.raw_data) // 4  # Each point is 4 bytes
        self.vehicle_data[vehicle_id]["last_state"]["lidar_points"] = points
        
        # More realistic proximity alert
        if points > LIDAR_PROXIMITY_THRESHOLD:
            self.vehicle_data[vehicle_id]["violation_flags"].add("proximity_alert")
        else:
            self.vehicle_data[vehicle_id]["violation_flags"].discard("proximity_alert")

    def _update_gps_data(self, vehicle_id, data):
        """Update GPS coordinates"""
        self.vehicle_data[vehicle_id]["last_state"]["gps"] = (data.latitude, data.longitude, data.altitude)

    def _update_imu_data(self, vehicle_id, data):
        """Update IMU data"""
        self.vehicle_data[vehicle_id]["last_state"]["imu"] = (
            data.accelerometer.x, data.accelerometer.y, data.accelerometer.z,
            data.gyroscope.x, data.gyroscope.y, data.gyroscope.z
        )

    def _handle_collision(self, vehicle_id, event):
        """Handle collision events"""
        self.vehicle_data[vehicle_id]["last_state"]["collision"] = {
            "partner": event.other_actor.type_id,
            "intensity": np.linalg.norm([event.normal_impulse.x, event.normal_impulse.y, event.normal_impulse.z])
        }
        
        self._log_data(vehicle_id, True)

    def _handle_lane_invasion(self, vehicle_id, event):
        """Handle lane invasion events"""
        self._log_data(vehicle_id, True)

    def _log_data(self, vehicle_id, is_violation):
        """Log vehicle data to CSV and Firebase with flag for violations"""
        current_time = time.time()
        
        # Skip if not enough time has passed since last collection
        if current_time - self.last_data_collection_time[vehicle_id] < DATA_COLLECTION_INTERVAL:
            return
            
        # For violations, check if we've already logged it recently
        if is_violation:
            last_time = self.last_violation_time[vehicle_id].get("general", 0)
            if current_time - last_time < MIN_VIOLATION_INTERVAL:
                return
            self.last_violation_time[vehicle_id]["general"] = current_time
            
        # Balance the dataset - skip some non-violations if we have too many
        if not is_violation:
            if self.non_violation_counts[vehicle_id] > self.violation_counts[vehicle_id] * (1/VIOLATION_RATIO - 1):
                return
            self.non_violation_counts[vehicle_id] += 1
        else:
            self.violation_counts[vehicle_id] += 1
            
        self.last_data_collection_time[vehicle_id] = current_time
        
        # Get vehicle state
        vehicle = self.vehicle_data[vehicle_id]["vehicle"]
        control = vehicle.get_control()
        velocity = vehicle.get_velocity()
        speed = 3.6 * np.linalg.norm([velocity.x, velocity.y, velocity.z])
        transform = vehicle.get_transform()
        
        # Get sensor data
        state = self.vehicle_data[vehicle_id]["last_state"]
        gps_data = state.get("gps", (0, 0, 0))
        imu_data = state.get("imu", (0, 0, 0, 0, 0, 0))
        lidar_points = state.get("lidar_points", 0)
        proximity_alert = "proximity_alert" in self.vehicle_data[vehicle_id]["violation_flags"]
        collision_partner = state.get("collision", {}).get("partner", "None")
        
        # Prepare CSV row
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        event_id = str(uuid.uuid4())[:8]
        
        row = [
            event_id, timestamp, vehicle_id, int(is_violation),
            speed, control.throttle, control.brake, control.steer, transform.rotation.yaw,
            *gps_data, *imu_data,
            lidar_points, int(proximity_alert), collision_partner,
            self._get_traffic_light_state(vehicle),
            self._get_lane_type(vehicle)
        ]
        
        # Write to CSV
        with self.csv_lock:
            with open(self.csv_file, "a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(row)
        
        # Write to Firebase if initialized
        if self.firebase_initialized:
            try:
                data_record = {
                    "timestamp": timestamp,
                    "vehicle_id": vehicle_id,
                    "is_violation": is_violation,
                    "speed_kph": speed,
                    "throttle": control.throttle,
                    "brake": control.brake,
                    "steer": control.steer,
                    "heading": transform.rotation.yaw,
                    "location": {
                        "latitude": gps_data[0],
                        "longitude": gps_data[1],
                        "altitude": gps_data[2]
                    },
                    "imu": {
                        "accel_x": imu_data[0],
                        "accel_y": imu_data[1],
                        "accel_z": imu_data[2],
                        "gyro_x": imu_data[3],
                        "gyro_y": imu_data[4],
                        "gyro_z": imu_data[5]
                    },
                    "lidar_points": lidar_points,
                    "proximity_alert": proximity_alert,
                    "collision_partner": collision_partner,
                    "traffic_light_state": str(self._get_traffic_light_state(vehicle)),
                    "lane_type": str(self._get_lane_type(vehicle))
                }
                
                ref = db.reference('traffic_data')
                new_record_ref = ref.push()
                new_record_ref.set(data_record)
                print(f"Firebase update successful for vehicle {vehicle_id}")
                
            except Exception as e:
                print(f"Firebase write error: {str(e)}")
                # Optionally implement retry logic here
        
        if is_violation:
            print(f"\U0001F6A8 {timestamp} - Vehicle {vehicle_id}: Violation detected")
        else:
            print(f"\U0001F697 {timestamp} - Vehicle {vehicle_id}: Normal operation")

    def _get_traffic_light_state(self, vehicle):
        """Get current traffic light state for vehicle"""
        light = vehicle.get_traffic_light()
        return light.state if light and light.state != carla.TrafficLightState.Unknown else "None"

    def _get_lane_type(self, vehicle):
        """Get current lane type for vehicle"""
        waypoint = world.get_map().get_waypoint(vehicle.get_location())
        return waypoint.lane_type if waypoint else "None"

    def monitor_traffic(self):
        """Main monitoring loop for traffic data collection"""
        while True:
            world.tick()  # Advance simulation in synchronous mode
            
            for vehicle_id, data in self.vehicle_data.items():
                vehicle = data["vehicle"]
                velocity = vehicle.get_velocity()
                speed = 3.6 * np.linalg.norm([velocity.x, velocity.y, velocity.z])
                current_time = time.time()
                
                # Check for violations
                violation_detected = False
                
                # Speeding violation
                if speed > MAX_SPEED_KPH:
                    violation_detected = True
                
                # Red light violation
                light = vehicle.get_traffic_light()
                if not violation_detected and light and light.state == carla.TrafficLightState.Red:
                    if speed > 5:  # Moving toward red light
                        violation_detected = True
                
                # Wrong way detection
                waypoint = world.get_map().get_waypoint(vehicle.get_location())
                if not violation_detected and waypoint:
                    lane_dir = waypoint.transform.get_forward_vector()
                    vehicle_dir = vehicle.get_transform().get_forward_vector()
                    
                    if lane_dir.dot(vehicle_dir) < -0.7:  # ~135 degree difference
                        violation_detected = True
                
                # Log data (either violation or normal operation)
                if violation_detected:
                    self._log_data(vehicle_id, True)
                else:
                    # Only log non-violation data if we're maintaining the ratio
                    if self.non_violation_counts[vehicle_id] <= self.violation_counts[vehicle_id] * (1/VIOLATION_RATIO - 1):
                        self._log_data(vehicle_id, False)

    def cleanup(self):
        """Cleanup all spawned actors"""
        for vehicle_id, data in self.vehicle_data.items():
            for sensor in data["sensors"].values():
                sensor.destroy()
            data["vehicle"].destroy()
        print("All vehicles and sensors destroyed")

if __name__ == "__main__":
    try:
        collector = TrafficDataCollector()
        collector.spawn_vehicles(20)
        
        # Start monitoring in separate thread
        monitor_thread = threading.Thread(target=collector.monitor_traffic)
        monitor_thread.daemon = True
        monitor_thread.start()
        
        # Main thread just sleeps until interrupted
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
        collector.cleanup()