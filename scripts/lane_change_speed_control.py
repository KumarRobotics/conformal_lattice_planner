#!/usr/bin/python3

from math import exp
import numpy as np
import matplotlib.pyplot as plt

from intelligent_driver_model import adaptive_cruise_control as idm

# Data for a vehicle.
vehicle_dtype = np.dtype([
    ('x',      'float'),
    ('y',      'float'),
    ('theta',  'float'),
    ('speed',  'float'),
    ('accel',  'float'),
    ('lead',   'int'  ),
    ('policy', 'float'),
    ('length', 'float')])

# Data for a waypoint on the path.
waypoint_dtype = np.dtype([
    ('s',     'float'),
    ('x',     'float'),
    ('y',     'float'),
    ('theta', 'float'),
    ('kappa', 'float')])

def interpolate_path(path, distance):

    if distance<path[0]['s'] or distance>path[-1]['s']:
        raise ValueError('Invalid input distance:{}, path start:{} path end:{}'.format(
                         distance, path[0]['s'], path[-1]['s']))

    if distance==path[ 0]['s']: return path[ 0]
    if distance==path[-1]['s']: return path[-1]

    # Find the left and right precomputed waypoints,
    # which will be used in the interpolation.
    ridx = np.argmax(path['s']>distance)
    lidx = ridx - 1;

    # Weights for the left and right waypoints.
    lw =  (path[ridx]['s']-distance) / (path[ridx]['s']-path[lidx]['s'])
    rw = -(path[lidx]['s']-distance) / (path[ridx]['s']-path[lidx]['s'])

    # Compute the target waypoint.
    # FIXME: Is there an easier way to do this?
    waypoint = np.array([(
           path[lidx]['s']    *lw + path[ridx]['s']    *rw,
           path[lidx]['x']    *lw + path[ridx]['x']    *rw,
           path[lidx]['y']    *lw + path[ridx]['y']    *rw,
           path[lidx]['theta']*lw + path[ridx]['theta']*rw,
           path[lidx]['kappa']*lw + path[ridx]['kappa']*rw)], dtype=waypoint_dtype)

    return waypoint

def following_distance(leader, follower):
    return leader['x']-follower['x'] - (leader['length']+follower['length'])/2.0

def check_collision(snapshot):

    for vehicle in snapshot:
        if vehicle['lead'] >= snapshot.size: continue
        if following_distance(snapshot[vehicle['lead']], vehicle) <= 0.0: return True
    return False

def vehicle_accels1(snapshot, ego_path):
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

def lc_idm(ego_v, ego_v0, progress, cl_v=None, cl_s=None, tl_v=None, tl_s=None, tf_v=None, tf_s=None):

    inv_sigmoid = lambda x : 1.0 - exp(-5.0*x)

    if progress >= 0.5:
        return idm(ego_v, ego_v0, tl_v, tl_s)

    cl_accel, cl_w = 0.0, 0.0
    tl_accel, tl_w = 0.0, 0.0
    tf_accel, tf_w = 0.0, 0.0

    progress *= 2.0

    if (cl_v is not None) and (cl_s is not None):
        cl_accel = idm(ego_v, ego_v0, cl_v, cl_s)
        cl_w = (1.0-inv_sigmoid(progress)) * (1.0-progress*(1.0-progress))

    if (tl_v is not None) and (tl_s is not None):
        tl_accel = idm(ego_v, ego_v0, tl_v, tl_s)
        tl_w = inv_sigmoid(progress)

    if (tf_v is not None) and (tf_s is not None):
        # FIXME: The policy speed of the target lane follower is not known.
        if (tl_v is None) or (tl_s is None):
            tf_accel = idm(tf_v, tf_v) - idm(tf_v, tf_v, ego_v, tf_s)
        else:
            tf_accel = idm(tf_v, tf_v, tl_v, tl_s+tf_s) - idm(tf_v, tf_v, ego_v, tf_s)
        tf_w = (1.0-inv_sigmoid(progress)) * (progress*(1.0-progress))

    return cl_accel*cl_w + tl_accel*tl_w + tf_accel*tf_w

def vehicle_accels2(snapshot, ego_path):
    accels = np.zeros(snapshot.size)
    # TODO: Generate the ego vehicle acceleration.
    accels[0] = lc_idm(
            snapshot[0]['speed'], snapshot[0]['policy'],
            snapshot[0]['y']/3.7,
            snapshot[1]['speed'], following_distance(snapshot[1], snapshot[0]),
            snapshot[2]['speed'], following_distance(snapshot[2], snapshot[0]),
            snapshot[3]['speed'], following_distance(snapshot[0], snapshot[3]))

    # Generate the acceleration for the rest of the vehicles.
    for i in range(1, snapshot.size):
        vehicle = snapshot[i]
        if vehicle['lead'] >= snapshot.size:
            accels[i] = idm(vehicle['speed'], vehicle['policy'])
        else:
            accels[i] = idm(vehicle['speed'], vehicle['policy'],
                            snapshot[vehicle['lead']]['speed'],
                            following_distance(snapshot[vehicle['lead']], vehicle))
    return accels

def simulate_traffic(initial_snapshot, ego_path, controller):
    # Resolution of the time in the simulation.
    time_res = 0.05

    # Distance of the ego on the lane changing path.
    ego_distance_on_path = 0.0

    # Stores the snapshot at each time instance.
    snapshots = []
    snapshot = initial_snapshot

    for t in np.arange(0.0, 20.0, time_res):
        accels = controller(snapshot, ego_path)
        snapshot['accel'] = accels
        snapshots.append((t, np.copy(snapshot)))

        # Update the position and speed of all agents.
        for i in range(1, snapshot.size):
            snapshot[i]['x']     += snapshot[i]['speed']*time_res +\
                                    snapshot[i]['accel']*time_res*time_res*0.5
            snapshot[i]['speed'] += snapshot[i]['accel']*time_res

        # Update the state and speed of the ego.
        # It's a bit tricky since the ego is following the lane changing path.
        ego_distance_on_path += snapshot[0]['speed']*time_res +\
                                snapshot[0]['accel']*time_res*time_res*0.5

        # If the ego has reached or exceeded the end of the path, stop the simulation.
        if ego_distance_on_path >= ego_path[-1]['s']: break

        ego_waypoint = interpolate_path(ego_path, ego_distance_on_path)
        snapshot[0]['x']     = ego_waypoint[0]['x']
        snapshot[0]['y']     = ego_waypoint[0]['y']
        snapshot[0]['theta'] = ego_waypoint[0]['theta']
        snapshot[0]['speed'] = snapshot[0]['speed'] + snapshot[0]['accel']*time_res

        # Update the lead for the ego and v3 if necessary.
        if snapshot[0]['y'] >= 3.7/2.0:
            snapshot[0]['lead'] = 2
            snapshot[3]['lead'] = 0

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

    # Plot ego acceleration and speed.
    ax_accel.plot(t, ego['accel'], label=controller_label)
    ax_speed.plot(t, ego['speed'], label=controller_label)

    # Plot the following distance.
    ego_following_distance = np.zeros(t.size)
    for i in range(0, t.size):
        if ego[i]['lead'] < snapshots[0][1].size:
            ego_following_distance[i] = following_distance(vehicles[ego[i]['lead']][i], ego[i])
        else:
            ego_following_distance[i] = np.nan

    ax_distance.plot(t, ego_following_distance, label=controller_label)
    return

def case_comparison():

    # Load the lane chaning path of the ego vehicle.
    ego_path = np.loadtxt('left_lane_change_path', waypoint_dtype)

    # Initialize the micro-traffic.
    # +x: right; +y: up
    # v0: the ego vehicle
    # v1: lead on the current lane
    # v2: lead on the target lane
    # v3: follower on the target lane
    # The setup of the vehicles is the following:
    # --v3---------------------------------v2---------
    # --------ego-------------v1----------------------
    #snapshot = np.array([
    #    (  0.0, 0.0, 0.0, 23.0, 0.0,  1, 25.0, 4.7),
    #    ( 30.0, 0.0, 0.0, 20.0, 0.0, 10, 20.0, 4.7),
    #    ( 60.0, 3.7, 0.0, 20.0, 0.0, 10, 20.0, 4.7),
    #    (-20.0, 3.7, 0.0, 24.0, 0.0,  2, 25.0, 4.7) ], dtype=vehicle_dtype)
    snapshot = np.array([
        (  0.0, 0.0, 0.0, 25.0, 0.0,  1, 30.0, 4.7),
        ( 30.0, 0.0, 0.0, 20.0, 0.0, 10, 20.0, 4.7),
        ( 50.0, 3.7, 0.0, 20.0, 0.0, 10, 20.0, 4.7),
        (-10.0, 3.7, 0.0, 25.0, 0.0,  2, 30.0, 4.7) ], dtype=vehicle_dtype)

    # Simulate the traffic.
    snapshots_lf = simulate_traffic(np.copy(snapshot), ego_path, vehicle_accels1)
    snapshots_lc = simulate_traffic(np.copy(snapshot), ego_path, vehicle_accels2)

    # Generate plots
    fig_accel,    ax_accel    = plt.subplots()
    fig_speed,    ax_speed    = plt.subplots()
    fig_distance, ax_distance = plt.subplots()

    draw_snapshots(snapshots_lf, ax_accel, ax_speed, ax_distance, 'LF-IDM')
    draw_snapshots(snapshots_lc, ax_accel, ax_speed, ax_distance, 'LC-IDM')

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

def switch_accel_wrt(snapshots, variable, ego_path, title):

    switch_accel_lf = np.zeros(variable.size, dtype=[('ego', 'float'), ('tf', 'float')])
    switch_accel_lc = np.zeros(variable.size, dtype=[('ego', 'float'), ('tf', 'float')])

    for i, snapshot in enumerate(snapshots):
        initial_snapshot = np.copy(snapshot)

        try:
            snapshots_lf = simulate_traffic(np.copy(initial_snapshot), ego_path, vehicle_accels1)
            t_lf, vehicles_lf = parse_snapshots(snapshots_lf)
            ego_switch_lane_idx = np.nonzero(vehicles_lf[0]['y']>=3.7/2.0)[0][0]
            switch_accel_lf['ego'][i] = vehicles_lf[0]['accel'][ego_switch_lane_idx-1]
            switch_accel_lf['tf' ][i] = vehicles_lf[3]['accel'][ego_switch_lane_idx+1]
        except:
            print('Collision detected for LF-IDM.')
            switch_accel_lf['ego'][i] = np.nan
            switch_accel_lf['tf' ][i] = np.nan

        try:
            snapshots_lc = simulate_traffic(np.copy(initial_snapshot), ego_path, vehicle_accels2)
            t_lc, vehicles_lc = parse_snapshots(snapshots_lc)
            ego_switch_lane_idx = np.nonzero(vehicles_lc[0]['y']>=3.7/2.0)[0][0]
            switch_accel_lc['ego'][i] = vehicles_lc[0]['accel'][ego_switch_lane_idx-1]
            switch_accel_lc['tf' ][i] = vehicles_lc[3]['accel'][ego_switch_lane_idx+1]
        except:
            print('Collision detected for LC-IDM.')
            switch_accel_lc['ego'][i] = np.nan
            switch_accel_lc['tf' ][i] = np.nan

    fig, ax = plt.subplots()
    ax.plot(variable, switch_accel_lf['ego'], label='ego LF-IDM')
    ax.plot(variable, switch_accel_lf['tf' ], label='TF LF-IDM' )
    ax.plot(variable, switch_accel_lc['ego'], label='ego LC-IDM')
    ax.plot(variable, switch_accel_lc['tf' ], label='TF LC-IDM' )
    ax.set_title('Switch Accel w.r.t. ' + title)
    ax.set_xlabel(title)
    ax.grid()
    ax.legend()

    return fig, ax

def switch_accel_comparison():
    # Load the lane chaning path of the ego vehicle.
    ego_path = np.loadtxt('left_lane_change_path', waypoint_dtype)

    tl_distance_axis = np.linspace(20.0, 80.0, 61)
    tf_distance_axis = np.linspace(-60, -10, 51)
    tf_speed_axis    = np.linspace(10.0, 30.0, 21)

    # Initialize the micro-traffic.
    # +x: right; +y: up
    # v0: the ego vehicle
    # v1: lead on the current lane
    # v2: lead on the target lane
    # v3: follower on the target lane
    # The setup of the vehicles is the following:
    # --v3---------------------------------v2---------
    # --------ego-------------v1----------------------
    snapshot = np.array([
        (  0.0, 0.0, 0.0, 25.0, 0.0,  1, 30.0, 4.7),
        ( 30.0, 0.0, 0.0, 20.0, 0.0, 10, 20.0, 4.7),
        ( 50.0, 3.7, 0.0, 20.0, 0.0, 10, 20.0, 4.7),
        (-20.0, 3.7, 0.0, 25.0, 0.0,  2, 30.0, 4.7) ], dtype=vehicle_dtype)

    # Switch accel w.r.t. tl distance.
    snapshots = []
    for tl_distance in tl_distance_axis:
        initial_snapshot = np.copy(snapshot)
        initial_snapshot[2]['x'] = tl_distance
        snapshots.append(initial_snapshot)

    fig_tl_distance, ax_tl_distance = switch_accel_wrt(snapshots, tl_distance_axis, ego_path, 'TL Distance')

    # Switch accel w.r.t. tf distance.
    snapshots = []
    for tf_distance in tf_distance_axis:
        initial_snapshot = np.copy(snapshot)
        initial_snapshot[3]['x'] = tf_distance
        snapshots.append(initial_snapshot)

    fig_tf_distance, ax_tf_distance = switch_accel_wrt(snapshots, tf_distance_axis, ego_path, 'TF Distance')

    # Switch accel w.r.t. tf speed.
    snapshots = []
    for tf_speed in tf_speed_axis:
        initial_snapshot = np.copy(snapshot)
        initial_snapshot[3]['speed'] = tf_speed
        snapshots.append(initial_snapshot)

    fig_tf_speed, ax_tf_speed = switch_accel_wrt(snapshots, tf_speed_axis, ego_path, 'TF Speed')

    plt.show()
    return


if __name__ == '__main__':
    #case_comparison()
    switch_accel_comparison()
