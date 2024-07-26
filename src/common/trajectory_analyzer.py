#!/usr/bin/python3

import numpy as np


class TrajectoryPoint:
    x_m = 0.0
    y_m = 0.0
    theta_rad = 0.0
    kappa = 0.0
    s_m = 0.0
    dkappa = 0.0
    v_mps = 0.0
    a_mpss = 0.0


class TrajectoryAnalyzer:
    def __init__(self, file_path=None):
        self.first_frame = True
        self.index = 0
        self.trajectory_points = []
        if file_path == None:
            self.__CreateLine()
            # self.__CreatCircle()
            print("trajectory create!")
        else:
            try:
                with open(file_path, "r") as f:
                    for line in f:

                        if line[0:2] != "x_":
                            continue
                        x_m, y_m, theta_rad, kappa, s_m, dkappa, v_mps, a_mpss = (
                            line.strip().split(",")
                        )
                        point = TrajectoryPoint()
                        point.x_m = float(x_m.split(":")[1].strip())
                        point.y_m = float(y_m.split(":")[1].strip())
                        point.theta_rad = float(theta_rad.split(":")[1].strip())
                        point.kappa = float(kappa.split(":")[1].strip())
                        point.s_m = float(s_m.split(":")[1].strip())
                        point.dkappa = float(dkappa.split(":")[1].strip())
                        point.v_mps = float(v_mps.split(":")[1].strip())
                        point.a_mpss = float(a_mpss.split(":")[1].strip())

                        self.trajectory_points.append(point)
            except (FileNotFoundError, IsADirectoryError):
                self.__CreatCircle()
                print("trajectory file not found, creat default circle path")

    def QueryNearestPointByPosition(self, x, y):
        d_min = self.__PointDistanceSquare(self.trajectory_points[0], x, y)
        n = len(self.trajectory_points)

        for i in (
            range(n)
            if self.first_frame
            else range(
                max(0, self.index - 1000),
                min(len(self.trajectory_points), self.index + 1000),
            )
        ):
            dis_new = self.__PointDistanceSquare(self.trajectory_points[i], x, y)
            if d_min > dis_new:
                d_min = dis_new
                self.index = i

        self.first_frame = False
        index = self.index

        # TODO: Need interpolation
        return self.trajectory_points[index], index

    def QueryNearestPointByS(self, target):
        left = max(0, self.index - 1000)
        right = min(len(self.trajectory_points), self.index + 5000)
        while left <= right:
            mid = (left + right) // 2
            if self.trajectory_points[mid].s_m == target:
                return self.trajectory_points[mid], mid
            elif self.trajectory_points[mid].s_m < target:
                left = mid + 1
            else:
                right = mid - 1
        # TODO: Need interpolation
        return self.trajectory_points[left], left
    

    def __search_position(self, trajectory_points, target):
        left = max(0, self.index - 200)
        right = min(len(self.trajectory_points), self.index + 1000)
        while left <= right:
            mid = (left + right) // 2
            if trajectory_points[mid].s_m == target:
                return mid
            elif trajectory_points[mid].s_m < target:
                left = mid + 1
            else:
                right = mid - 1
        return left

    def __PointDistanceSquare(self, point, x, y):
        return (point.x_m - x) ** 2 + (point.y_m - y) ** 2

    def __CreatCircle(self):
        t = np.linspace(-np.pi, np.pi, 10000)
        R = 50
        for i in range(t.size):
            point = TrajectoryPoint()
            point.x_m = R * np.cos(t[i])
            point.y_m = R * np.sin(t[i])
            point.theta_rad = t[i] + np.pi / 2.0
            point.kappa = 1.0 / R
            point.dkappa = 0.0
            point.a_mpss = 0.0
            point.v_mps = 10.0
            point.s_m = 2 * np.pi * R * i / t.size

            self.trajectory_points.append(point)

    def __CreateLine(self):
        t = np.linspace(0, 1000.0, 20000)
        for i in range(t.size):
            point = TrajectoryPoint()
            point.x_m = 0.0
            point.y_m = t[i]
            point.theta_rad = np.pi / 2.0
            point.kappa = 0.0
            point.dkappa = 0.0
            point.a_mpss = 0.0
            point.v_mps = 10.0
            point.s_m = t[i]

            self.trajectory_points.append(point)