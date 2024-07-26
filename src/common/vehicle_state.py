#!/usr/bin/python3
from common.vehicle_param import VehicleParam
import numpy as np


class VehicleState:
    def __init__(self) -> None:
        self.X = 0.0
        self.Y = 0.0
        self.v_lon = 0.0
        self.v_lat = 0.0
        self.acc = 0.0
        self.jerk = 0.0
        self.heading = 0.0
        self.yaw_rate = 0.0
        self.steer_wheel_angle = 0.0
        self.steer_rate = 0.0

    def InitVehicleStateFromPoint(self, point):
        vehicle_param = VehicleParam()
        v = point.v_mps
        ref_curvature = point.kappa
        lf = vehicle_param.lf
        lr = vehicle_param.lr
        cr = vehicle_param.cr
        wheelbase = vehicle_param.wheelbase
        mass = vehicle_param.mass
        sideslip_angle = (
            lr * ref_curvature - lf * mass * v * v * ref_curvature / cr / wheelbase
        )

        self.X = point.x_m
        self.Y = point.y_m
        self.v_lon = point.v_mps
        self.acc = point.a_mpss
        self.heading = point.theta_rad + lr * ref_curvature - sideslip_angle
        self.yaw_rate = point.kappa * point.v_mps
        self.steer_wheel_angle = (
            self.yaw_rate
            * vehicle_param.wheelbase
            * (1.0 + vehicle_param.k * point.v_mps * point.v_mps)
            / point.v_mps
            * vehicle_param.steer_ratio
            * 180.0
            / np.pi
        )
        self.v_lat = v * np.tan(sideslip_angle)

    def GetPositionCOM(self):
        vehicle_param = VehicleParam()
        dX = vehicle_param.lr * np.cos(self.heading)
        dY = vehicle_param.lr * np.sin(self.heading)
        X_com = self.X + dX
        Y_com = self.Y + dY
        return X_com, Y_com

    def GetPositionRearAxleCenter(self, X_com, Y_com):
        vehicle_param = VehicleParam()
        dX = -vehicle_param.lr * np.cos(self.heading)
        dY = -vehicle_param.lr * np.sin(self.heading)
        X_rear = X_com + dX
        Y_rear = Y_com + dY
        return X_rear, Y_rear
