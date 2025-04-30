import sys
import os
import time
import carla
from datetime import datetime
import inspect  # For debugging

# Debugging setup
DEBUG = True
def debug_print(*args):
    if DEBUG:
        caller = inspect.currentframe().f_back
        print(f"[DEBUG] {caller.f_code.co_name}:", *args)

# ========== Configuration ==========
CARLA_ROOT = r"C:\Users\lazya\OneDrive\Documents\sem 4\ml\CARLA_Latest"
SCENARIO_RUNNER_PATH =r'C:\Users\lazya\OneDrive\Documents\sem 4\ml\CARLA_Latest\scenario_runner-0.9.15\scenario_runner-0.9.15'

# Add to Python path
sys.path.append(SCENARIO_RUNNER_PATH)
sys.path.append(os.path.join(CARLA_ROOT, "WindowsNoEditor", "PythonAPI", "carla"))

try:
    from srunner.scenariomanager.scenario_manager import ScenarioManager
    from srunner.scenarioconfigs.scenario_configuration import ScenarioConfiguration
    from srunner.scenarios.follow_leading_vehicle import FollowLeadingVehicle
    print("Scenario Runner imports successful")
except ImportError as e:
    print(f"Import error: {e}")
    sys.exit(1)

class TrafficViolationDetector:
    def __init__(self):
        self.client = None
        self._world = None
        self.vehicles = []
        debug_print("Detector initialized")

    @property
    def world(self):
        """Validated world access"""
        if self._world is None:
            debug_print("World access attempted before initialization")
            raise RuntimeError("World not initialized")
        debug_print(f"World accessed: {self._world}")
        return self._world

    def connect_to_carla(self):
        """Enhanced connection with debugging"""
        debug_print("Attempting connection")
        try:
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(30.0)
            debug_print("Client created", self.client)

            # Get world with validation
            world = self.client.get_world()
            debug_print("World retrieved", world)
            
            if world is None:
                raise RuntimeError("Received None world")
            
            # Wait for stabilization
            for _ in range(5):
                try:
                    _ = world.get_map()
                    break
                except RuntimeError:
                    time.sleep(1)
            
            self._world = world
            debug_print("World validated and set", self._world)
            
            # Find vehicles with debug info
            self.vehicles = [a for a in world.get_actors() 
                           if a.type_id.startswith('vehicle.')]
            debug_print(f"Found {len(self.vehicles)} vehicles", 
                       [v.id for v in self.vehicles[:3]], 
                       "..." if len(self.vehicles) > 3 else "")
            
            return True
            
        except Exception as e:
            debug_print("Connection failed", str(e))
            return False

    def setup_scenario(self):
        """Debug-enabled scenario setup"""
        debug_print("Starting scenario setup")
        try:
            # Debug world state
            debug_print("World state check", 
                       f"World exists: {hasattr(self, '_world')}", 
                       f"World value: {getattr(self, '_world', None)}")
            
            if not hasattr(self, '_world') or self._world is None:
                raise RuntimeError("World not initialized")
            
            if not self.vehicles:
                raise RuntimeError("No vehicles available")
            
            # Select ego vehicle with debug
            ego_vehicle = next((v for v in self.vehicles if 'tesla' in v.type_id), self.vehicles[0])
            debug_print("Selected ego vehicle", 
                       f"ID: {ego_vehicle.id}", 
                       f"Type: {ego_vehicle.type_id}",
                       f"Location: {ego_vehicle.get_location()}")

            spawn_point = ego_vehicle.get_transform()
            debug_print("Spawn point", spawn_point)

            # Scenario config with debug
            config = ScenarioConfiguration()
            config.town = 'Town01'  # Note: Using Town01 even if map is Town10HD
            config.name = 'FollowLeadingVehicle'
            config.trigger_points = [spawn_point]
            debug_print("Scenario config created", vars(config))

            # Initialize Scenario Manager with debug
            scenario_manager = ScenarioManager(debug_mode=True)
            debug_print("ScenarioManager created")

            # Debug world before scenario creation
            debug_print("World before scenario creation", 
                       f"Type: {type(self.world)}",
                       f"ID: {id(self.world)}",
                       f"Map: {self.world.get_map().name}")

            # Create scenario
            scenario = FollowLeadingVehicle(self.world, [ego_vehicle], config)
            debug_print("Scenario created")

            scenario_manager.load_scenario(scenario)
            debug_print("Scenario loaded")

            return scenario_manager
            
        except Exception as e:
            debug_print("Scenario setup failed", str(e))
            debug_print("Current state", 
                       f"World: {getattr(self, '_world', None)}",
                       f"Vehicles: {[v.id for v in self.vehicles] if hasattr(self, 'vehicles') else 'None'}")
            raise

    # ... [rest of the methods remain the same as previous solution]

if __name__ == "__main__":
    print("=== Traffic Violation Detection ===")
    detector = TrafficViolationDetector()
    
    try:
        if not detector.connect_to_carla():
            raise RuntimeError("Failed to connect to CARLA")
            
        scenario_manager = detector.setup_scenario()
        detector.monitor_traffic(scenario_manager)
        
    except Exception as e:
        print(f"\n[ERROR] Fatal error: {str(e)}")
        debug_print("Final state", 
                   f"World: {getattr(detector, '_world', None)}",
                   f"Client: {getattr(detector, 'client', None)}")
    finally:
        detector.cleanup()
        print("\n[COMPLETE] Script finished")