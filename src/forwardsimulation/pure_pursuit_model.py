import numpy as np
import matplotlib.pyplot as plt

from common.vehicle_param import VehicleParam
from common.vehicle_state import VehicleState

class PurePursuitModel:
    def __init__(self, vehicle_param, lookahead_distance = 5.0):
        self.wheelbase = vehicle_param.wheelbase
        self.lookahead_distance = lookahead_distance

    def compute_steering_angle(self, vehicle_state, target_point):
        """
        计算纯追踪算法的转向角度
        :param vehicle_state X Y v_lon v_lat acc jerk heading yaw_rate steer_wheel_angle steer_rate
        :param target_point: x_m y_m theta_rad kappa s_m dkappa v_mps a_mpss
        :return: 转向角度 δ
        """

        X, Y, heading = vehicle_state.X, vehicle_state.Y, vehicle_state.heading
        Xt, Yt = target_point[0].x_m, target_point[0].y_m
        # 计算转向角度 α
        dx = Xt - X
        dy = Yt - Y
        alpha = np.arctan2(dy, dx) - heading

        Ld = (dx ** 2 + dy ** 2) ** 0.5

        # 计算实际转向角度 δ
        steering_angle = np.arctan2(2 * self.wheelbase * np.sin(alpha), Ld)
        # print(f"dx: {dx}, dy: {dy}, alpha: {alpha}, Ld: {Ld}, steering_angle: {steering_angle}")
        return steering_angle

    