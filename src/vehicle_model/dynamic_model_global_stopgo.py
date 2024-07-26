#!/usr/bin/python3

from src.common.vehicle_param import VehicleParam
from src.common.vehicle_state import VehicleState

import numpy as np
import copy

from collections import deque

# Body Coordinate x-forward, y-left
# 在这个车辆模型中，X Y 代表 COM 位置的坐标
# cf cr = -cf -cr
class DynamicModelStopgo:
    def __init__(self, vehicle_state=None, vehicle_param=None) -> None:
        self.name = "Dynamic Model stopgo"
        self.time = 0.0
        if vehicle_state == None:
            self.vehicle_state = VehicleState()
        else:
            self.vehicle_state = copy.deepcopy(vehicle_state)

        if vehicle_param == None:
            self.vehicle_param = VehicleParam()
        else:
            self.vehicle_param = vehicle_param

        self.last_steer_wheel_angle = self.vehicle_state.steer_wheel_angle
        self.last_acc = self.vehicle_state.acc

        self.is_pure_delay = False
        self.buffer_size = 10
        self.delay_buffer = deque([0.0] * self.buffer_size, maxlen=self.buffer_size)

    def __UpdateNextState(self, matrix_state, control_cmd, dt):
        v_lon = matrix_state[0]
        v_lat = matrix_state[1]
        heading = matrix_state[2]
        yaw_rate = matrix_state[3]
        X = matrix_state[4]
        Y = matrix_state[5]

        a_k = control_cmd[0]
        delta_k = control_cmd[1]

        cf = - self.vehicle_param.cf
        cr = - self.vehicle_param.cr
        mass = self.vehicle_param.mass
        iz = self.vehicle_param.iz
        lf = self.vehicle_param.lf
        lr = self.vehicle_param.lr

        # Compute the next state variables
        X_next = X + dt * (v_lon * np.cos(heading) - v_lat * np.sin(heading))
        Y_next = Y + dt * (v_lon * np.sin(heading) + v_lat * np.cos(heading))
        heading_next = heading + dt * yaw_rate
        v_lon_next = v_lon + dt * a_k

        denominator_v = mass * v_lon - dt * (cf + cr)
        if abs(denominator_v) < 1e-5:
            v_lat_next = v_lat  # Prevent division by zero
        else:
            v_lat_next = (mass * v_lon * v_lat + dt * (lf * cf -lr * cr) * yaw_rate - dt * cf * delta_k * v_lon - dt * mass * v_lon**2 * yaw_rate) / denominator_v

        denominator_yaw_rate = iz * v_lon - dt * (lf**2 * cf +lr**2 * cr)
        if abs(denominator_yaw_rate) < 1e-5:
            yaw_rate_next = yaw_rate  # Prevent division by zero
        else:
            yaw_rate_next = (iz * v_lon * yaw_rate + dt * (lf * cf -lr * cr) * v_lat - dt * lf * cf * delta_k * v_lon) / denominator_yaw_rate
        
        heading_next = self.__NormalizeAngle(heading_next)
        yaw_rate_next = self.__NormalizeAngle(yaw_rate_next)

        # print(f"denominator_v = {denominator_v}, denominator_yaw_rate = {denominator_yaw_rate}")
        # print([v_lon_next, v_lat_next, heading_next, yaw_rate_next, X_next, Y_next])
        return np.array([v_lon_next, v_lat_next, heading_next, yaw_rate_next, X_next, Y_next])

    def Step(self, new_time, target_steer, target_acc):
        dt = new_time - self.time
        self.time = new_time
        if dt < 0.001:
            print("Step interval is less than 1 ms, No Update!")
            return self.vehicle_state

        # Update actuators' response
        self.vehicle_state.acc = (
            self.vehicle_state.acc * (1.0 - dt / self.vehicle_param.acc_delay_tau)
            + target_acc * dt / self.vehicle_param.acc_delay_tau
        )

        # pure delay
        if self.is_pure_delay:
            self.delay_buffer.append(target_steer)
            self.vehicle_state.steer_wheel_angle = self.delay_buffer[0]
        else:
            self.vehicle_state.steer_wheel_angle = (
                self.vehicle_state.steer_wheel_angle
                * (1.0 - dt / self.vehicle_param.steer_delay_tau)
                + target_steer * dt / self.vehicle_param.steer_delay_tau
            )
        # print(f"steer_wheel_angle: {self.vehicle_state.steer_wheel_angle}")
        # self.vehicle_state.steer_wheel_angle = target_steer

        self.vehicle_state.steer_rate = (
            self.vehicle_state.steer_wheel_angle - self.last_steer_wheel_angle
        ) / dt
        self.vehicle_state.jerk = (self.vehicle_state.acc - self.last_acc) / dt

        self.last_acc = self.vehicle_state.acc
        self.last_steer_wheel_angle = self.vehicle_state.steer_wheel_angle

        # Dynamic model
        steer_deg2tire_rad_factor = np.pi / 180.0 / self.vehicle_param.steer_ratio
        tire_rad = (
            self.vehicle_state.steer_wheel_angle + self.vehicle_param.steer_offset
        ) * steer_deg2tire_rad_factor

        matrix_state = np.array(
                [
                    self.vehicle_state.v_lon,
                    self.vehicle_state.v_lat,
                    self.vehicle_state.heading,
                    self.vehicle_state.yaw_rate,
                    self.vehicle_state.X,
                    self.vehicle_state.Y,
                ]
            )

        control_cmd = np.array(
            [self.vehicle_state.acc,
             tire_rad, 
            ]
        )
        matrix_state_next = self.__UpdateNextState(matrix_state, control_cmd, dt)
        
        self.vehicle_state.v_lon = matrix_state_next[0]
        self.vehicle_state.v_lat = matrix_state_next[1]
        self.vehicle_state.heading = matrix_state_next[2]
        self.vehicle_state.yaw_rate = matrix_state_next[3]
        self.vehicle_state.X = matrix_state_next[4]
        self.vehicle_state.Y = matrix_state_next[5]

        return self.vehicle_state

    def GetMatrixState(self):
        matrix_state = np.array(
            [
                self.vehicle_state.v_lon,
                self.vehicle_state.v_lat,
                self.vehicle_state.heading,
                self.vehicle_state.yaw_rate,
                self.vehicle_state.X,
                self.vehicle_state.Y,
            ]
        )
        return matrix_state

    def GetMatrixControl(self):
        matrix_control = np.array(
            [
                self.vehicle_state.steer_wheel_angle,
                self.vehicle_state.acc,
            ]
        )
        return matrix_control

    def __NormalizeAngle(self, angle):
        a = np.mod(angle + np.pi, 2 * np.pi)
        if a < 0.0:
            a += 2 * np.pi
        return a - np.pi