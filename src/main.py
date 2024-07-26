import os
import sys
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from utils.path_utils import *
from common.vehicle_param import VehicleParam
from common.vehicle_state import VehicleState
from common.trajectory_analyzer import TrajectoryAnalyzer
from forwardsimulation.forward_sim_controller import ForwardSimController
from vehicle_model.dynamic_model_geoloc import DynamicModelGeoloc

def generate_referenceline_boundary():
    # generate reference line
    key_pt_xs = [0, 0]
    key_pt_ys = [0, 40]
    ref_path_x_list, ref_path_y_list, ref_path_yaw_list, \
        ref_path_kappa_list, ref_path_s_list = \
        generate_cublic_spline_reference_line(key_pt_xs, key_pt_ys, ds=1.0)

    return ref_path_x_list, ref_path_y_list, ref_path_yaw_list, ref_path_kappa_list, ref_path_s_list

def get_referenceline_from_trajectory_analyzer(trajectory_analyzer):
    ref_path_x_list = []
    ref_path_y_list = []
    ref_path_yaw_list = []
    ref_path_kappa_list = []
    ref_path_s_list = []
    for point in trajectory_analyzer.trajectory_points:
        ref_path_x_list.append(point.x_m)
        ref_path_y_list.append(point.y_m)
        ref_path_yaw_list.append(point.theta_rad)
        ref_path_kappa_list.append(point.kappa)
        ref_path_s_list.append(point.s_m)
    return ref_path_x_list, ref_path_y_list, ref_path_yaw_list, ref_path_kappa_list, ref_path_s_list


def generate_forwardsim():
    # ref_path_x_list, ref_path_y_list, ref_path_yaw_list, \
    #     ref_path_kappa_list, ref_path_s_list = generate_referenceline_boundary()
    
    trajectory_analyzer = TrajectoryAnalyzer()
    ref_path_x_list, ref_path_y_list, ref_path_yaw_list, \
        ref_path_kappa_list, ref_path_s_list = get_referenceline_from_trajectory_analyzer(trajectory_analyzer)
    vehicle_state = VehicleState()
    vehicle_state.InitVehicleStateFromPoint(
        trajectory_analyzer.trajectory_points[0]
    )
    vehicle_state.X -= 1.0
    ts = 0.02
    model1 = DynamicModelGeoloc(vehicle_state)
    controller1 = ForwardSimController(
        trajectory_analyzer,
        model1.vehicle_state,
        model1.name,
        ts,
    )
    
    veh_x = []
    veh_y = []
    for i in range(1000):
        request_steer_deg, request_acc_mpss = controller1.ComputeControlCommand()
        # print("request_steer_deg: ", request_steer_deg)

        model1.Step(i*ts, request_steer_deg, request_acc_mpss)
        # print("vehicle_state: ", model1.vehicle_state.X, model1.vehicle_state.Y, model1.vehicle_state.heading)
        veh_x.append(model1.vehicle_state.X)
        veh_y.append(model1.vehicle_state.Y)
    
    ax.clear()
    ax.plot(ref_path_x_list, ref_path_y_list, label='Centerline')
    ax.plot(veh_x, veh_y, label='vehicle 1')

    # Assuming lane width is 3.5 meters
    lane_width = 3.5
    left_boundary_x = ref_path_x_list - lane_width / 2 * np.sin(ref_path_yaw_list)
    left_boundary_y = ref_path_y_list + lane_width / 2 * np.cos(ref_path_yaw_list)
    right_boundary_x = ref_path_x_list + lane_width / 2 * np.sin(ref_path_yaw_list)
    right_boundary_y = ref_path_y_list - lane_width / 2 * np.cos(ref_path_yaw_list)

    # Plotting the boundaries
    ax.plot(left_boundary_x, left_boundary_y, label='Left Boundary', linestyle='--')
    ax.plot(right_boundary_x, right_boundary_y, label='Right Boundary', linestyle='--')

    # Adding labels and legend
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Lane Centerline and Boundaries')
    ax.legend()
    ax.grid(True)
    ax.axis('equal')
    

if __name__ == "__main__":
    
    # Plotting the centerline
    fig, ax = plt.subplots(figsize=(12, 12))
    plt.subplots_adjust(left=0.25, bottom=0.25)
    ax.axis("equal")
    generate_forwardsim()
    
    plt.show()
