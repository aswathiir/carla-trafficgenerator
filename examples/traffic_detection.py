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
# Connect to CARLA
client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()

# Constants
MAX_SPEED_KPH = 50  # Speed limit for violations
MIN_VIOLATION_INTERVAL = 5  # Seconds between logging same violation type

class TrafficViolationDetector:
    def __init__(self):
        self.vehicles = []
        self.vehicle_data = {}
        self.last_violation_time = defaultdict(dict)
        # Add Firebase initialization
        self.firebase_initialized = False
        self._init_firebase()
        

        # CSV setup
        self.csv_file = "traffic_violations_detailed.csv"
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
                    "Event_ID", "Timestamp", "Vehicle_ID", "Violation_Type", "Details",
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
        except Exception as e:
            print(f"Failed to initialize Firebase: {str(e)}")
            self.firebase_initialized = False
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
                        "violation_flags": set()
                    }
                    self.vehicles.append(vehicle)
                    self.attach_sensors(vehicle_id, vehicle)
            except Exception as e:
                print(f"Error spawning vehicle {i}: {str(e)}")
        
        print(f"Spawned {len(self.vehicles)} vehicles")

    def attach_sensors(self, vehicle_id, vehicle):
        """Attach sensors to vehicle with proper configuration"""
        try:
            # LiDAR
            lidar_bp = blueprint_library.find("sensor.lidar.ray_cast")
            lidar_bp.set_attribute("range", "50")
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
        """Process LiDAR data"""
        points = len(data.raw_data) // 4
        self.vehicle_data[vehicle_id]["last_state"]["lidar_points"] = points
        
        # Simple proximity alert
        if points > 500:
            self.vehicle_data[vehicle_id]["violation_flags"].add("proximity_alert")

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
        violation_type = "Collision"
        details = f"Collision with {event.other_actor.type_id}"
        
        self.vehicle_data[vehicle_id]["last_state"]["collision"] = {
            "partner": event.other_actor.type_id,
            "intensity": np.linalg.norm([event.normal_impulse.x, event.normal_impulse.y, event.normal_impulse.z])
        }
        
        self._log_violation(vehicle_id, violation_type, details)

    def _handle_lane_invasion(self, vehicle_id, event):
        """Handle lane invasion events"""
        violation_type = "Lane Violation"
        details = "Crossed lane markings"
        self._log_violation(vehicle_id, violation_type, details)

    def _log_violation(self, vehicle_id, violation_type, details):
        """Log violation to CSV with throttling"""
        current_time = time.time()
        last_time = self.last_violation_time[vehicle_id].get(violation_type, 0)
        
        if current_time - last_time >= MIN_VIOLATION_INTERVAL:
            self.last_violation_time[vehicle_id][violation_type] = current_time
            
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
            
            # Prepare CSV row
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            event_id = str(uuid.uuid4())[:8]
            
            row = [
                event_id, timestamp, vehicle_id, violation_type, details,
                speed, control.throttle, control.brake, control.steer, transform.rotation.yaw,
                *gps_data, *imu_data,
                state.get("lidar_points", 0),
                "proximity_alert" in self.vehicle_data[vehicle_id]["violation_flags"],
                state.get("collision", {}).get("partner", "None"),
                self._get_traffic_light_state(vehicle),
                self._get_lane_type(vehicle)
            ]
            # Prepare data for Firebase
            violation_data = {
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "vehicle_id": vehicle_id,
                "violation_type": violation_type,
                "details": details,
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
                "lidar_points": state.get("lidar_points", 0),
                "proximity_alert": "proximity_alert" in self.vehicle_data[vehicle_id]["violation_flags"],
                "collision_partner": state.get("collision", {}).get("partner", "None"),
                "traffic_light_state": str(self._get_traffic_light_state(vehicle)),
                "lane_type": str(self._get_lane_type(vehicle))
            }
            
            # Write to Firebase if initialized
            if self.firebase_initialized:
                try:
                    ref = db.reference('violations')
                    new_violation_ref = ref.push()
                    new_violation_ref.set(violation_data)
                except Exception as e:
                    print(f"Failed to write to Firebase: {str(e)}")
            # Write to CSV
            with self.csv_lock:
                with open(self.csv_file, "a", newline="") as file:
                    writer = csv.writer(file)
                    writer.writerow(row)
            
            print(f"\U0001F6A8 {timestamp} - Vehicle {vehicle_id}: {violation_type} - {details}")

    def _get_traffic_light_state(self, vehicle):
        """Get current traffic light state for vehicle"""
        light = vehicle.get_traffic_light()
        return light.state if light else "None"

    def _get_lane_type(self, vehicle):
        """Get current lane type for vehicle"""
        waypoint = world.get_map().get_waypoint(vehicle.get_location())
        return waypoint.lane_type if waypoint else "None"

    def monitor_violations(self):
        """Main monitoring loop for violations"""
        while True:
            world.tick()  # Advance simulation in synchronous mode
            
            for vehicle_id, data in self.vehicle_data.items():
                vehicle = data["vehicle"]
                velocity = vehicle.get_velocity()
                speed = 3.6 * np.linalg.norm([velocity.x, velocity.y, velocity.z])
                
                # Speeding violation
                if speed > MAX_SPEED_KPH:
                    self._log_violation(
                        vehicle_id, 
                        "Speeding", 
                        f"Speed: {speed:.2f} km/h (Limit: {MAX_SPEED_KPH} km/h)"
                    )
                
                # Red light violation
                light = vehicle.get_traffic_light()
                if light and light.state == carla.TrafficLightState.Red:
                    if speed > 5:  # Moving toward red light
                        self._log_violation(
                            vehicle_id,
                            "Red Light Violation",
                            f"Approaching red light at {speed:.2f} km/h"
                        )
                
                # Wrong way detection
                waypoint = world.get_map().get_waypoint(vehicle.get_location())
                if waypoint:
                    lane_dir = waypoint.transform.get_forward_vector()
                    vehicle_dir = vehicle.get_transform().get_forward_vector()
                    
                    if lane_dir.dot(vehicle_dir) < -0.7:  # ~135 degree difference
                        self._log_violation(
                            vehicle_id,
                            "Wrong Way Driving",
                            "Vehicle moving opposite to lane direction"
                        )

    def cleanup(self):
        """Cleanup all spawned actors"""
        for vehicle_id, data in self.vehicle_data.items():
            for sensor in data["sensors"].values():
                sensor.destroy()
            data["vehicle"].destroy()
        print("All vehicles and sensors destroyed")

if __name__ == "__main__":
    try:
        detector = TrafficViolationDetector()
        detector.spawn_vehicles(20)
        
        # Start monitoring in separate thread
        monitor_thread = threading.Thread(target=detector.monitor_violations)
        monitor_thread.daemon = True
        monitor_thread.start()
        
        # Main thread just sleeps until interrupted
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
        detector.cleanup()