#!/usr/bin/python3

from math import exp
import numpy as np
import matplotlib.pyplot as plt

from intelligent_driver_model import adaptive_cruise_control as idm
from intelligent_driver_model import distance_gap
from intelligent_driver_model import time_gap
from intelligent_driver_model import saturate_accel

# Data for a vehicle.
vehicle_dtype = np.dtype([
    ('x',       'float'),
    ('y',       'float'),
    ('theta',   'float'),
    ('speed',   'float'),
    ('accel',   'float'),
    ('lead',    'int'  ),
    ('policy',  'float'),
    ('length',  'float')])

# Data for a waypoint on the path.
waypoint_dtype = np.dtype([
    ('s',     'float'),
    ('x',     'float'),
    ('y',     'float'),
    ('theta', 'float'),
    ('kappa', 'float')])

# The LQR speed control policy.
# The first element is the control policy for lane keep free run,
# while the second is for lane keep with a lead vehicle.
lqr_policies = [np.zeros(11), np.zeros(11)]

def solve_are(A, B, Q, R, K0, N):
    K = K0
    for i in range(0, int(N)):
        P = np.linalg.inv(np.transpose(B).dot(K).dot(B)+R)
        S = (K.dot(B)).dot(P).dot(np.transpose(B).dot(K))
        K = np.transpose(A).dot(K-S).dot(A) + Q

    return K

def solve_lqr(q, r, T, dt):

    # The nonhomogeneous linear system is described by:
    # x' = Ax + Bu + Fy
    # where y is the exogeneous disturbance.
    A = np.array([
        [1.0, 0.0, 0.0, -dt],
        [0.0, 1.0, 0.0, -dt],
        [0.0, 0.0, 1.0,  dt],
        [0.0, 0.0, 0.0, 1.0]])
    B = np.array(
        [0.0, 0.0, 0.0, 1.0])
    F = np.array([
        [ dt, 0.0, 0.0],
        [0.0,  dt, 0.0],
        [0.0, 0.0, -dt],
        [0.0, 0.0, 0.0]])

    # Augment the original system with reference and exogeneous disturbance
    AA = np.zeros((11, 11))
    AA[0:4,  0:4]  = A
    AA[0:4,  4:8]  = A - np.eye(4)
    AA[0:4,  8:11] = F
    AA[4:8,  4:8]  = np.eye(4)
    AA[8:11, 8:11] = np.eye(3)

    BB = np.zeros((11, 1))
    BB[0:4, 0] = B

    # Construct the full matrices, Q and R.
    Q = np.zeros((11, 11))
    Q[0:4, 0:4] = np.diag(q)
    R = np.diag(r)

    # Solve for the optimal feedback gain of finite horizon.
    K = solve_are(AA, BB, Q, R, np.zeros((11, 11)), T/dt)
    P = np.linalg.inv(np.transpose(BB).dot(K).dot(BB)+R)
    L = -P.dot(np.transpose(BB).dot(K).dot(AA))

    return L

def lqr(ego_v, ego_v0, lead_v=None, s=None):

    # Form the state vector.
    x = np.zeros((11, 1))


    if (lead_v is None) or (s is None):
        x[3, 0] = ego_v - ego_v0
        x[7, 0] = ego_v0
        L = lqr_policies[0]
    else:
        x[3, 0] = ego_v - lead_v
        x[7, 0] = lead_v
        x[0, 0] = s - (distance_gap+lead_v*time_gap)
        x[4, 0] = distance_gap + lead_v*time_gap
        x[8, 0] = lead_v
        L = lqr_policies[1]

    accel = L.dot(x)
    return saturate_accel(accel)

def following_distance(leader, follower):
    return leader['x']-follower['x'] - (leader['length']+follower['length'])/2.0

def check_collision(snapshot):
    for vehicle in snapshot:
        if vehicle['lead'] >= snapshot.size: continue
        if following_distance(snapshot[vehicle['lead']], vehicle) <= 0.0: return True
    return False

def vehicle_accels1(snapshot):
    accels = np.zeros(snapshot.size)
    for i in range(0, snapshot.size):
        vehicle = snapshot[i]
        if vehicle['lead'] >= snapshot.size:
            accels[i] = idm(vehicle['speed'], vehicle['policy'])
        else:
            accels[i] = idm(vehicle['speed'], vehicle['policy'],
                            snapshot[vehicle['lead']]['speed'],
                            following_distance(snapshot[vehicle['lead']], vehicle))
    return accels

def vehicle_accels2(snapshot):
    accels = np.zeros(snapshot.size)

    ego = snapshot[0]
    if ego['lead'] >= snapshot.size:
        accels[0] = lqr(ego['speed'], ego['policy'])
    else:
        accels[0] = lqr(ego['speed'], ego['policy'],
                        snapshot[ego['lead']]['speed'],
                        following_distance(snapshot[ego['lead']], ego))

    for i in range(1, snapshot.size):
        vehicle = snapshot[i]
        if vehicle['lead'] >= snapshot.size:
            accels[i] = idm(vehicle['speed'], vehicle['policy'])
        else:
            accels[i] = idm(vehicle['speed'], vehicle['policy'],
                            snapshot[vehicle['lead']]['speed'],
                            following_distance(snapshot[vehicle['lead']], vehicle))
    return accels

def simulate_traffic(initial_snapshot, controller):
    time_res = 0.05
    snapshots = []
    snapshot = initial_snapshot

    for t in np.arange(0.0, 50.0, time_res):
        accels = controller(snapshot)
        snapshot['accel'] = accels
        snapshots.append((t, np.copy(snapshot)))

        for i in range(snapshot.size):
            snapshot[i]['x']     += snapshot[i]['speed']*time_res + \
                                    snapshot[i]['accel']*time_res*time_res*0.5
            snapshot[i]['speed'] += snapshot[i]['accel']*time_res

            if check_collision(snapshot):
                print('Collision snapshot: \n', snapshot)
                raise RuntimeError('Collision detected during the simulation.')

    return snapshots

def parse_snapshots(snapshots):
    time     = np.zeros(len(snapshots))
    vehicles = [np.zeros(len(snapshots), dtype=vehicle_dtype)
                for i in range(0, snapshots[0][1].size)]

    for i, frame in enumerate(snapshots):
        t, snapshot = frame
        time[i] = t
        for j in range(snapshot.size):
            vehicles[j][i] = snapshot[j]

    return time, vehicles


def draw_snapshots(snapshots, ax_accel, ax_speed, ax_distance, controller_label):

    t, vehicles = parse_snapshots(snapshots)
    ego = vehicles[0]

    # Plot ego acceleration.
    ax_accel.plot(t, ego['accel'], label=controller_label)
    # Plot the ego speed.
    ax_speed.plot(t, ego['speed'], label=controller_label)

    # Plot the ego following distance.
    ego_following_distance = np.zeros(t.size)
    for i in range(0, t.size):
        if ego[i]['lead'] < snapshots[0][1].size:
            ego_following_distance[i] = following_distance(vehicles[ego[i]['lead']][i], ego[i])
        else:
            ego_following_distance[i] = np.nan

    ax_distance.plot(t, ego_following_distance, label=controller_label)
    return

def case_comparison():

    # Initialize the micro-traffic.
    # +x: right
    # v0: the ego vehicle
    # v1: lead on the current lane
    # The setup of the vehicles is the following:
    # --------ego-------------v1----------------------
    snapshot = np.array([
        ( 0.0, 0.0, 0.0, 25.0, 0.0,  1, 25.0, 4.7),
        (20.0, 0.0, 0.0, 18.0, 0.0, 10, 18.0, 4.7) ], dtype=vehicle_dtype)
    # --------ego--------------------------------------
    #snapshot = np.array([
    #    (0.0, 0.0, 0.0, 30.0, 0.0, 10, 15.0, 4.7) ], dtype=vehicle_dtype)

    # Simulate the traffic with all vehicles controlled by IDM.
    snapshots_idm = simulate_traffic(np.copy(snapshot), vehicle_accels1)
    snapshots_lqr = simulate_traffic(np.copy(snapshot), vehicle_accels2)

    # Generate plots
    fig_accel,    ax_accel    = plt.subplots()
    fig_speed,    ax_speed    = plt.subplots()
    fig_distance, ax_distance = plt.subplots()

    draw_snapshots(snapshots_idm, ax_accel, ax_speed, ax_distance, 'IDM')
    draw_snapshots(snapshots_lqr, ax_accel, ax_speed, ax_distance, 'LQR')

    ax_accel.set_xlabel('t(s)')
    ax_accel.set_ylabel('a(m/s/s)')
    ax_accel.set_title('Ego Acceleration')
    ax_accel.legend()
    ax_accel.grid()

    ax_speed.set_xlabel('t(s)')
    ax_speed.set_ylabel('v(m/s)')
    ax_speed.set_title('Ego Speed')
    ax_speed.legend()
    ax_speed.grid()

    ax_distance.set_xlabel('t(s)')
    ax_distance.set_ylabel('s(m)')
    ax_distance.set_title('Ego Following Distance')
    ax_distance.legend()
    ax_distance.grid()

    plt.show()
    return

def minmax_accel_comparison():

    distance_axis   = np.arange(20.0, 100.0, 5.0)
    lead_speed_axis = np.arange(1.0, 29.0, 1.0)

    max_accel_idm = np.zeros((distance_axis.size, lead_speed_axis.size))
    min_accel_idm = np.zeros((distance_axis.size, lead_speed_axis.size))
    max_accel_lqr = np.zeros((distance_axis.size, lead_speed_axis.size))
    min_accel_lqr = np.zeros((distance_axis.size, lead_speed_axis.size))

    for i, distance in enumerate(distance_axis):
        for j, speed in enumerate(lead_speed_axis):
            print('distance:{} lead speed:{}'.format(distance, speed))
            # Create the initial snapshot.
            snapshot = np.array([
                (0.0, 0.0, 0.0, 20.0, 0.0, 1, 30.0, 4.7),
                (distance, 0.0, 0.0, speed, 0.0, 10, speed, 4.7) ], dtype=vehicle_dtype)

            # Simulate the traffic.
            try:
                snapshots_idm = simulate_traffic(np.copy(snapshot), vehicle_accels1)
                t_idm, vehicles_idm = parse_snapshots(snapshots_idm)
                ego_idm = vehicles_idm[0]
                max_accel_idm[i, j] = np.max(ego_idm['accel'])
                min_accel_idm[i, j] = np.min(ego_idm['accel'])
            except:
                print('Collision detected with IDM')
                max_accel_idm[i, j] = np.nan
                min_accel_idm[i, j] = np.nan

            try:
                snapshots_lqr = simulate_traffic(np.copy(snapshot), vehicle_accels2)
                t_lqr, vehicles_lqr = parse_snapshots(snapshots_lqr)
                ego_lqr = vehicles_lqr[0]
                max_accel_lqr[i, j] = np.max(ego_lqr['accel'])
                min_accel_lqr[i, j] = np.min(ego_lqr['accel'])
            except:
                print('Collision detected with LQR')
                max_accel_lqr[i, j] = np.nan
                min_accel_lqr[i, j] = np.nan


    # Compute the difference.
    accel_idm = max_accel_idm
    accel_idm[accel_idm < 0.0] = 0.0

    brake_idm = -min_accel_idm
    brake_idm[brake_idm < 0.0] = 0.0

    accel_lqr = max_accel_lqr
    accel_lqr[accel_lqr < 0.0] = 0.0

    brake_lqr = -min_accel_lqr
    brake_lqr[brake_lqr < 0.0] = 0.0

    diff_max_accel = accel_idm - accel_lqr
    diff_min_accel = brake_idm - brake_lqr

    diff_max_accel[np.isnan(diff_max_accel)] = 0.0
    diff_min_accel[np.isnan(diff_min_accel)] = 0.0

    fig_max, ax_max = plt.subplots()
    fig_min, ax_min = plt.subplots()

    im_max = ax_max.imshow(diff_max_accel, origin='lower')
    im_min = ax_min.imshow(diff_min_accel, origin='lower')

    cbar_max = ax_max.figure.colorbar(im_max, ax=ax_max)
    cbar_min = ax_min.figure.colorbar(im_min, ax=ax_min)
    #cbar.ax.set_ylabel(cbarlabel, rotation=-90, va="bottom")

    ax_max.set_xticks(np.arange(lead_speed_axis.size))
    ax_max.set_yticks(np.arange(distance_axis.size))
    ax_max.set_xticklabels(lead_speed_axis)
    ax_max.set_yticklabels(distance_axis)
    ax_max.set_xlabel('speed(m/s)')
    ax_max.set_ylabel('distance(m)')
    ax_max.set_title('IDM(max)-LQR(max)')

    ax_min.set_xticks(np.arange(lead_speed_axis.size))
    ax_min.set_yticks(np.arange(distance_axis.size))
    ax_min.set_xticklabels(lead_speed_axis)
    ax_min.set_yticklabels(distance_axis)
    ax_min.set_xlabel('speed(m/s)')
    ax_min.set_ylabel('distance(m)')
    ax_min.set_title('IDM(min)-LQR(min)')

    plt.show()
    return

def main():
    np.set_printoptions(precision=3, linewidth=130)

    # Solve the LQR speed controller
    global lqr_polices
    lqr_policies[0] = solve_lqr(np.array([0.0, 0.0, 0.0, 1.0]), np.array([23.0]), 30.0, 0.1)
    lqr_policies[1] = solve_lqr(np.array([0.1, 0.0, 0.0, 20.0]), np.array([100.0]), 30.0, 0.1)

    #case_comparison()
    minmax_accel_comparison()
    return


if __name__ == '__main__':
    main()
