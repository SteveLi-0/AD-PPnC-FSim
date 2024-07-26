from common.vehicle_param import VehicleParam
from common.vehicle_state import VehicleState

import math

class Observation:
    def __init__(self, vehicle_model_list) -> None:
        self.vehicle_state_list = [model.vehicle_state for model in vehicle_model_list]

    def GetAllIDMInfo(self):
        all_idm_info = []
        for vehicle in self.vehicle_state_list:
            min_distance, relative_speed, leading_vehicle_state = \
            self._find_leading_vehicle(vehicle, self.vehicle_state_list)
            all_idm_info.append(
                {
                    "min_distance": min_distance,
                    "relative_speed": relative_speed,
                    "leading_vehicle_state": leading_vehicle_state
                }
            )
        return all_idm_info
    
    def GetIDMInfo(self, vehicle_state):
        min_distance, relative_speed, leading_vehicle_state = \
        self._find_leading_vehicle(vehicle_state, self.vehicle_state_list)
        idm_info = {
                "min_distance": min_distance,
                "relative_speed": relative_speed,
                "leading_vehicle_state": leading_vehicle_state
            }
        return idm_info

    def _find_leading_vehicle(self, current_vehicle, vehicle_state_list):
        min_distance = float('inf')
        relative_speed = 0
        leading_vehicle_state = None
        for vehicle in vehicle_state_list:
            if vehicle != current_vehicle and self._is_in_front(current_vehicle, vehicle):
                distance,  dv \
                    = self._calculate_distance_and_relative_speed(current_vehicle, vehicle)
                if distance < min_distance:  
                    min_distance = distance
                    relative_speed = dv
                    leading_vehicle_state = vehicle
        if min_distance == float('inf'):
            min_distance = 1000.0
        else:
            min_distance -= 3.0
        return min_distance, relative_speed, leading_vehicle_state 
    
    def _is_in_front(self, current_vehicle, other_vehicle, lon_threshold=50, lat_threshold=1.5):
        dx = other_vehicle.X - current_vehicle.X
        dy = other_vehicle.Y - current_vehicle.Y
        
        # Convert the heading to radians
        heading_rad = current_vehicle.heading
        
        # Transform other_vehicle position to current_vehicle coordinate system
        local_x = dx * math.cos(heading_rad) + dy * math.sin(heading_rad)
        local_y = -dx * math.sin(heading_rad) + dy * math.cos(heading_rad)
        
        angle_to_other = math.degrees(math.atan2(dy, dx))
        angle_diff = abs(angle_to_other - current_vehicle.heading)
        
        # Check if the other vehicle is within the rectangular area in front of the current vehicle
        if 0 <= local_x <= lon_threshold and abs(local_y) <= lat_threshold:
            return True
        return False

    def _calculate_distance_and_relative_speed(self, vehicle1, vehicle2):
        dx = vehicle2.X - vehicle1.X
        dy = vehicle2.Y - vehicle1.Y
        distance = math.sqrt(dx**2 + dy**2)
        relative_speed = vehicle1.v_lon - vehicle2.v_lon
        return distance, relative_speed
