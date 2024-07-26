#!/usr/bin/python3


class VehicleParam:
    def __init__(self) -> None:
        self.wheelbase = 3.0
        self.steer_ratio = 12.6
        self.acc_delay_tau = 0.2
        self.steer_delay_tau = 0.15
        self.cf = 148969
        self.cr = 177616
        self.mass_fl = 525
        self.mass_fr = 525
        self.mass_rl = 550
        self.mass_rr = 550
        self.mass_f = self.mass_fl + self.mass_fr
        self.mass_r = self.mass_rl + self.mass_rr
        self.mass = self.mass_f + self.mass_r
        self.lf = self.wheelbase * (1.0 - self.mass_f / self.mass)
        self.lr = self.wheelbase * (1.0 - self.mass_r / self.mass)
        self.iz = self.lf * self.lf * self.mass_f + self.lr * self.lr * self.mass_r

        self.k = 0.0002851
        self.steer_offset = 0.0

    def ConvertTireRadToSteeringDeg(self, tire_rad):
        return tire_rad * 180 / 3.1415926 * self.steer_ratio

    def ConvertSteeringDegToTireRad(self, steer_angle):
        return steer_angle / 180 * 3.1415926 / self.steer_ratio
