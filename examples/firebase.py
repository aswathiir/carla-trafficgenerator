import firebase_admin
from firebase_admin import credentials, db
import sys
import os
import csv
import time
import threading
import uuid
import numpy as np
import carla
from collections import defaultdict
class TrafficViolationDetector:
    def __init__(self):
        # Initialize Firebase
        self.firebase_initialized = False
        self._init_firebase()
        
        # Rest of your existing initialization code...
    
    def _init_firebase(self):
        """Initialize Firebase connection"""
        try:
            # Download your Firebase service account key JSON from Firebase console
            cred = credentials.Certificate("serviceAccountKey.json")
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'https://carla-4f285-default-rtdb.firebaseio.com/'
            })
            self.firebase_initialized = True
            print("Firebase initialized successfully")
        except Exception as e:
            print(f"Failed to initialize Firebase: {str(e)}")
            self.firebase_initialized = False
    
    def _log_violation(self, vehicle_id, violation_type, details):
        """Log violation to CSV and Firebase with throttling"""
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
            
            # Rest of your existing CSV logging code...
            print(f"\U0001F6A8 {violation_data['timestamp']} - Vehicle {vehicle_id}: {violation_type} - {details}")