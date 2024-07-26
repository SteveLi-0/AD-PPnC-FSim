#!/usr/bin/python3

from src.common.vehicle_param import VehicleParam
from src.common.vehicle_state import VehicleState

import numpy as np
import copy


# Body Coordinate x-forward, y-left
class DynamicModelDr:
    def __init__(self, vehicle_state=None, vehicle_param=None) -> None:
        self.name = "Dynamic Model Dr"
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

    def __UpdateStateDot(self, matrix_state, tire_rad):
        v_lon = matrix_state[0]
        v_lat = matrix_state[1]
        heading = matrix_state[2]
        yaw_rate = matrix_state[3]
        X = matrix_state[4]
        Y = matrix_state[5]
        v_normal = np.sqrt(v_lon * v_lon + v_lat * v_lat)
        cf = self.vehicle_param.cf
        cr = self.vehicle_param.cr
        mass = self.vehicle_param.mass
        iz = self.vehicle_param.iz
        lf = self.vehicle_param.lf
        lr = self.vehicle_param.lr

        matrix_a = np.array(
            [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [
                    0.0,
                    -(cf + cr) / mass / v_lon,
                    0.0,
                    -v_lon - (cf * lf - cr * lr) / mass / v_lon,
                    0.0,
                    0.0,
                ],
                [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                [
                    0.0,
                    -(lf * cf - lr * cr) / iz / v_lon,
                    0.0,
                    -(lf * lf * cf + lr * lr * cr) / iz / v_lon,
                    0.0,
                    0.0,
                ],
                [
                    np.cos(heading),
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                [
                    np.sin(heading),
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            ]
        )

        matrix_b = np.array(
            [
                [0.0, 1.0],
                [cf / mass, 0.0],
                [0.0, 0.0],
                [lf * cf / iz, 0.0],
                [0.0, 0.0],
                [0.0, 0.0],
            ]
        )

        matrix_control = np.array([tire_rad, self.vehicle_state.acc])

        return matrix_a @ matrix_state + matrix_b @ matrix_control

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
        # self.vehicle_state.steer_wheel_angle = (
        #     self.vehicle_state.steer_wheel_angle
        #     * (1.0 - dt / self.vehicle_param.steer_delay_tau)
        #     + target_steer * dt / self.vehicle_param.steer_delay_tau
        # )

        self.vehicle_state.steer_rate = (
            self.vehicle_state.steer_wheel_angle - self.last_steer_wheel_angle
        ) / dt
        self.vehicle_state.jerk = (self.vehicle_state.acc - self.last_acc) / dt

        self.last_acc = self.vehicle_state.acc
        self.last_steer_wheel_angle = self.vehicle_state.steer_wheel_angle

        # Dynamic model is unstable at low speed, because velocity is in the
        # denominator of formula. Thus kinematic model is used to keep simulation
        # stable when vehicle velocity is small.
        steer_deg2tire_rad_factor = np.pi / 180.0 / self.vehicle_param.steer_ratio
        tire_rad = (
            self.vehicle_state.steer_wheel_angle + self.vehicle_param.steer_offset
        ) * steer_deg2tire_rad_factor

        if self.vehicle_state.v_lon > 1.5:
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
            matrix_k1 = self.__UpdateStateDot(matrix_state, tire_rad)
            matrix_k2 = self.__UpdateStateDot(
                matrix_state + matrix_k1 * dt / 2.0, tire_rad
            )
            matrix_k3 = self.__UpdateStateDot(
                matrix_state + matrix_k2 * dt / 2.0, tire_rad
            )
            matrix_k4 = self.__UpdateStateDot(matrix_state + matrix_k3 * dt, tire_rad)

            matrix_state += (
                (matrix_k1 + 2.0 * matrix_k2 + 2.0 * matrix_k3 + matrix_k4) * dt / 6.0
            )

            self.vehicle_state.v_lon = matrix_state[0]
            self.vehicle_state.v_lat = matrix_state[1]
            self.vehicle_state.heading = matrix_state[2]
            self.vehicle_state.yaw_rate = matrix_state[3]
            self.vehicle_state.X = matrix_state[4]
            self.vehicle_state.Y = matrix_state[5]

        else:
            # TODO: Need more accurate kinematic model at low speed
            self.vehicle_state.yaw_rate = (
                np.tan(tire_rad)
                / self.vehicle_param.wheelbase
                * self.vehicle_state.v_lon
            )
            self.vehicle_state.heading += self.vehicle_state.yaw_rate * dt
            self.vehicle_state.v_lon += self.vehicle_state.acc * dt
            self.vehicle_state.X += (
                np.cos(self.vehicle_state.heading) * self.vehicle_state.v_lon * dt
            )
            self.vehicle_state.Y += (
                np.sin(self.vehicle_state.heading) * self.vehicle_state.v_lon * dt
            )

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
