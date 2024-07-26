from common.trajectory_analyzer import TrajectoryPoint
from common.vehicle_param import VehicleParam
from forwardsimulation.intelligent_driver_model import IntelligentDriverModel
from forwardsimulation.pure_pursuit_model import PurePursuitModel

import copy
import scipy.linalg
import numpy as np
import matplotlib.pyplot as plt

from scipy.interpolate import interp1d

class ForwardSimController:
    def __init__(
        self, trajectory_analyzer, vehicle_state, model_type, ts, state_observation):

        self.name = "Forward Simulation Controller"
        self.model_type = model_type
        self.trajectory_analyzer = copy.deepcopy(trajectory_analyzer)
        self.matched_point = TrajectoryPoint()
        self.vehicle_param = VehicleParam()
        self.vehicle_state = vehicle_state
        self.is_first_frame = True

        self.L = self.vehicle_param.lf + self.vehicle_param.lr
        self.lf = self.vehicle_param.lf
        self.lr = self.vehicle_param.lr
        self.cf = self.vehicle_param.cf
        self.cr = self.vehicle_param.cr
        self.m = self.vehicle_param.mass
        self.iz = self.vehicle_param.iz
        self.k =  - self.m * (self.lf / self.cr - self.lr / self.cf) / self.L / self.L

        self.ts = ts
        self.last_x1 = None
        self.acc_final_deg = vehicle_state.acc
        self.steer_final_deg = vehicle_state.steer_wheel_angle
        self.compute_time = 0

        self.lon_controller = IntelligentDriverModel(self.vehicle_param)
        self.lat_controller = PurePursuitModel(self.vehicle_param, lookahead_distance=5.0)
        self.observation = state_observation

        print("Init agent controller succeess!")

    def ComputeControlCommand(self):
        # latral control
        X0, Y0 = self.vehicle_state.X, self.vehicle_state.Y
        self.matched_point = self.trajectory_analyzer.QueryNearestPointByPosition(X0, Y0)
        self.target_point = self.trajectory_analyzer.QueryNearestPointByS(
            self.matched_point[0].s_m + self.lat_controller.lookahead_distance
        )
        self.steer_rad = self.lat_controller.compute_steering_angle(
            self.vehicle_state, self.target_point)

        self.steer_final_deg = self.vehicle_param.ConvertTireRadToSteeringDeg(self.steer_rad)

        # longitudinal control
        idm_info = self.observation.GetIDMInfo(self.vehicle_state)
        self.acc_final_deg = self.lon_controller.GetIDMAcc(
            idm_info["min_distance"],
            self.vehicle_state.v_lon,
            idm_info["relative_speed"])

        
        return self.steer_final_deg, self.acc_final_deg
        
        