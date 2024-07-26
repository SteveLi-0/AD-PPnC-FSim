import numpy as np
import matplotlib.pyplot as plt

from common.vehicle_param import VehicleParam

class IntelligentDriverModel:
    def __init__(self, vehicle_paramm, v0=20.0, T=1.5, a=1.5, b=1.5, delta=4, s0=2.0):
        """
        初始化IDM模型参数
        :param v0: 期望速度 (m/s)
        :param T: 安全时间间隔 (s)
        :param a: 最大加速度 (m/s^2)
        :param b: 舒适减速度 (m/s^2)
        :param delta: 加速度指数
        :param s0: 最小间距 (m)
        """
        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.delta = delta
        self.s0 = s0
        self.vehicle_param = vehicle_paramm

    def GetIDMAcc(self, s, v, dv):
        """
        计算加速度
        :param s: 车间距离 (m)
        :param v: 本车速度 (m/s)
        :param dv: 相对速度 (m/s)
        :return: 加速度 (m/s^2)
        """
        if s > 200.0:
            s = 1000.0
            dv = 0.0
        s_star = self.s0 + max(0, v * self.T + v * dv / (2 * np.sqrt(self.a * self.b)))
        acc = self.a * (1 - (v / self.v0) ** self.delta - (s_star / s) ** 2)
        return acc

def simulate_idm(idm, s0, v0, v_lead, t_max=100, dt=0.1):
    """
    模拟IDM模型
    :param idm: IDM模型实例
    :param s0: 初始车间距离 (m)
    :param v0: 初始速度 (m/s)
    :param v_lead: 前车速度 (m/s)
    :param t_max: 模拟时间 (s)
    :param dt: 时间步长 (s)
    :return: 时间、速度和车间距离
    """
    t = np.arange(0, t_max, dt)
    s = np.zeros_like(t)
    v = np.zeros_like(t)
    s[0] = s0
    v[0] = v0

    for i in range(1, len(t)):
        dv = v[i - 1] - v_lead
        s[i] = s[i - 1] + (v_lead - v[i - 1]) * dt
        v[i] = v[i - 1] + idm.acceleration(s[i - 1], v[i - 1], dv) * dt
        v[i] = max(0, v[i])  # 确保速度不为负值

    return t, s, v

if __name__ == "__main__":
    """
    测试IDM模型
    """
    # 初始化IDM模型
    idm = IntelligentDriverModel()

    # 设置初始条件
    initial_distance = 10.0  # 初始车间距离 (m)
    initial_speed = 0.0      # 初始速度 (m/s)
    leader_speed = 15.0      # 前车速度 (m/s)

    # 模拟IDM模型
    t, s, v = simulate_idm(idm, initial_distance, initial_speed, leader_speed)

    # 绘图
    plt.figure(figsize=(10, 5))
    plt.subplot(2, 1, 1)
    plt.plot(t, s, label='Distance to lead vehicle')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.plot(t, v, label='Speed of following vehicle')
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.legend()
    plt.tight_layout()
    plt.show()
