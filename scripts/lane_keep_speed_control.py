#!/usr/bin/python3

from math import exp
import numpy as np
import matplotlib.pyplot as plt

from intelligent_driver_model import adaptive_cruise_control as idm

# Data for a vehicle.
vehicle_dtype = np.dtype([
    ('x',       'float'),
    ('y',       'float'),
    ('theta',   'float'),
    ('speed',   'float'),
    ('accel',   'float'),
    ('policy',  'float'),
    ('length',  'float')])

# Data for a waypoint on the path.
waypoint_dtype = np.dtype([
    ('s',     'float'),
    ('x',     'float'),
    ('y',     'float'),
    ('theta', 'float'),
    ('kappa', 'float')])

def solve_are(A, B, Q, R, K0, N):
    K = K0
    for i in range(0, N):
        S = (K.dot(B)).dot(numpy.inv(B.dot(K).dot(B)+R)).dot(B.dot(K))
        K = np.transpose(A).dot(K-S).dot(A) + Q

    return K

def solve_optimal_speed_policy(Q, R, N, dt):

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

    # Augment the original system with reference and disturbance
    AA = np.zeros((12, 12))
    AA[0:4, 0:4]  = A
    AA[0:4, 4:8]  = A - np.eye(4)
    AA[0:4, 8:12] = F

    BB = np.zeros(12)
    BB[0:4] = B

    # Solve for the optimal feedback gain of finite horizon.
    K = solve_are(AA, BB, Q, R, np.zeros(12, 12), 10.0/dt)
    L = -numpy.inv(BB.dot(K).dot(BB)+R).dot(BB.dot(K).dot(AA))

    return L


def following_distance(leader, follower):
    return leader['x']-follower['x'] - (leader['length']+follower['length'])/2.0

def check_collision(snapshot):
    # The traffic contains the ego only.
    if snapshot.size == 1: return False

    # Distance betweeen the ego and v1.
    if following_distance(snapshot[1], snapshot[0]) > 0.0: return False
    return True

def vehicle_accels(snapshot):
    accels = np.zeros(snapshot.size)

    # The traffic contains the ego only.
    if snapshot.size == 1:
        accels[0] = idm(snapshot[0]['speed'], snapshot[0]['policy'])
    else:
        accels[1] = idm(snapshot[1]['speed'], snapshot[1]['policy'])
        accels[0] = idm(snapshot[0]['speed'], snapshot[0]['policy'],
                        snapshot[1]['speed'],
                        following_distance(snapshot[1], snapshot[0]))

    return accels

def simulate_traffic(initial_snapshot, controller):
    time_res = 0.05
    snapshots = []
    snapshot = initial_snapshot

    for t in np.arange(0.0, 15.0, time_res):
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
    t   = np.zeros(len(snapshots))
    ego = np.zeros(len(snapshots), dtype=vehicle_dtype)

    for i in range(0, len(snapshots)):
        t[i]   = snapshots[i][0]
        ego[i] = snapshots[i][1][0]

    if snapshots[0][1].size > 1:
        v1  = np.zeros(len(snapshots), dtype=vehicle_dtype)
        for i in range(0, len(snapshots)):
            v1[i] = snapshots[i][1][1]

    if snapshots[0][1].size > 1: return t, ego, v1
    else: return t, ego

def draw(snapshots):

    data = parse_snapshots(snapshots)
    t    = data[0]
    ego  = data[1]

    # Plot ego acceleration.
    fig_accel, ax_accel = plt.subplots()
    ax_accel.plot(t, ego['accel'], label='IDM')
    ax_accel.set_xlabel('t(s)')
    ax_accel.set_ylabel('a(m/s/s)')
    ax_accel.set_title('Ego Acceleration')
    ax_accel.grid()

    # Plot the ego speed.
    fig_speed, ax_speed = plt.subplots()
    ax_speed.plot(t, ego['speed'], label='IDM')
    ax_speed.set_xlabel('t(s)')
    ax_speed.set_ylabel('v(m/s)')
    ax_speed.set_title('Ego Speed')
    ax_speed.grid()

    # Plot the ego following distance.
    if len(data) > 2:
        v1 = data[2]
        fig_distance, ax_distance = plt.subplots()
        ax_distance.plot(t, v1['x']-ego['x'], label='IDM')
        ax_distance.set_xlabel('t(s)')
        ax_distance.set_ylabel('s(m)')
        ax_distance.set_title('Ego Following Distance')
        ax_distance.grid()

    return

def main():

    # Initialize the micro-traffic.
    # The setup of the vehicles is the following:
    # --------ego-------------v1----------------------
    # +x: right
    # v0: the ego vehicle
    # v1: lead on the current lane
    snapshot = np.array([
        ( 0.0, 0.0, 0.0, 23.0, 0.0, 25.0, 4.7),
        (40.0, 0.0, 0.0, 20.0, 0.0, 20.0, 4.7) ], dtype=vehicle_dtype)
    #snapshot = np.array([
    #    (0.0, 0.0, 0.0, 23.0, 0.0, 25.0, 4.7) ], dtype=vehicle_dtype)

    # Simulate the traffic.
    snapshots = simulate_traffic(np.copy(snapshot), vehicle_accels)

    draw(snapshots)
    plt.show()

    return


if __name__ == '__main__':
    main()
