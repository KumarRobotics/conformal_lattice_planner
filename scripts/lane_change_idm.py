#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

from math import sqrt
from math import tanh
import numpy as np

from intelligent_driver_model import intelligent_driver_model as idm

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

def interpolate_path(path, distance):

    if distance<path[0]['s'] or distance>path[-1]['s']:
        raise ValueError('Invalid input distance:{}, path start:{} path end:{}'.format(
                         distance, path[0]['s'], path[-1]['s']))

    if distance==path[0]['s']: return path[0]
    if distance==path[-1]['s']: return path[-1]

    # Find the left and right precomputed waypoints,
    # which will be used in the interpolation.
    ridx = np.argmax(path['s']>distance)
    lidx = ridx - 1;

    # Weights for the left and right waypoints.
    lw =  (path[ridx]['s']-distance) / (path[ridx]['s']-path[lidx]['s'])
    rw = -(path[lidx]['s']-distance) / (path[ridx]['s']-path[lidx]['s'])

    # Compute the target waypoint.
    # FIXME: Is there a easier way to do this?
    target_waypoint = np.zeros(1, dtype=waypoint_dtype)
    #target_waypoint[0]['s'] = path[lidx]['s']*lw + path[ridx]['s']*rw
    #target_waypoint[0]['x'] = path[lidx]['x']*lw + path[ridx]['x']*rw
    #target_waypoint[0]['y'] = path[lidx]['y']*lw + path[ridx]['y']*rw
    #target_waypoint[0]['theta'] = path[lidx]['theta']*lw + path[ridx]['theta']*rw
    #target_waypoint[0]['kappa'] = path[lidx]['kappa']*lw + path[ridx]['kappa']*rw
    target_waypoint[0] = path[lidx]['s']*lw + path[ridx]['s']*rw,\
                         path[lidx]['x']*lw + path[ridx]['x']*rw,\
                         path[lidx]['y']*lw + path[ridx]['y']*rw,\
                         path[lidx]['theta']*lw + path[ridx]['theta']*rw,\
                         path[lidx]['kappa']*lw + path[ridx]['kappa']*rw

    return target_waypoint[0]

def check_collision(snapshot):
    # Initial traffic setup.
    # --v3---------------------------------v2---------
    # --------ego-------------v1----------------------

    # Check ego and v1.
    if snapshot[1]['x']-snapshot[0]['x'] <= \
       (snapshot[1]['length']+snapshot[0]['length'])/2.0 : return True;

    # Check ego and v2.
    if snapshot[2]['x']-snapshot[0]['x'] <= \
       (snapshot[2]['length']+snapshot[0]['length'])/2.0 : return True;

    # Check v3 and v2.
    if snapshot[2]['x']-snapshot[3]['x'] <= \
       (snapshot[2]['length']+snapshot[3]['length'])/2.0 : return True;

    return False

def vehicle_accels(snapshot, ego_path, ego_distance_on_path):
    # Initial traffic setup.
    # --v3---------------------------------v2---------
    # --------ego-------------v1----------------------

    accels = np.zeros(4)

    # Acceleration for v1.
    accels[1] = idm(snapshot[1]['speed'], snapshot[1]['policy'])
    # Acceleration for v2.
    accels[2] = idm(snapshot[2]['speed'], snapshot[2]['policy'])

    if snapshot[0]['y'] > 3.7/2.0:
        # The ego is on the target lane.
        # Acceleration for v3.
        accels[3] = idm(snapshot[3]['speed'], snapshot[3]['policy'],
                        snapshot[0]['speed'], snapshot[0]['x']-snapshot[3]['x'])
        # Acceleration for the ego.
        accels[0] = idm(snapshot[0]['speed'], snapshot[0]['policy'],
                        snapshot[2]['speed'], snapshot[2]['x']-snapshot[0]['x'])
    else:
        # The ego is still on the current lane.
        # Acceleration for v3.
        accels[3] = idm(snapshot[3]['speed'], snapshot[3]['policy'],
                        snapshot[2]['speed'], snapshot[2]['x']-snapshot[3]['x'])
        # Acceleration for the ego.
        accels[0] = idm(snapshot[0]['speed'], snapshot[0]['policy'],
                        snapshot[1]['speed'], snapshot[1]['x']-snapshot[0]['x'])

    return accels

def main():

    #================ Configuration ======================
    # Load the lane chaning path of the ego vehicle.
    ego_path = np.loadtxt('left_lane_change_path', waypoint_dtype)

    # Initialize the micro-traffic.
    # The setup of the vehicles is the following:
    # --v3---------------------------------v2---------
    # --------ego-------------v1----------------------
    # +x: right; +y: up
    # v0: the ego vehicle
    # v1: lead on the current lane
    # v2: lead on the target lane
    # v3: follower on the target lane
    snapshot = np.array([
        (  0.0, 0.0, 0.0, 20.0, 0.0, 25.0, 4.7),
        ( 20.0, 0.0, 0.0, 20.0, 0.0, 20.0, 4.7),
        ( 40.0, 3.7, 0.0, 20.0, 0.0, 20.0, 4.7),
        (-15.0, 3.7, 0.0, 20.0, 0.0, 20.0, 4.7) ], dtype=vehicle_dtype)

    #=============== Simulation ==========================
    # The ego starts from the beginning of the path.
    ego_distance_on_path = 0.0

    # Resolution of the time in the simulation.
    time_res = 0.1

    snapshots = []

    for t in np.arange(0.0, 20.0, time_res):
        accels = vehicle_accels(snapshot, ego_path, ego_distance_on_path)
        snapshot['accel'] = accels
        snapshots.append(snapshot)

        print('t = {}\n'.format(t), snapshot, '\n')

        # Update the position and speed of all agents.
        for i in range(1, 3):
            snapshot[i]['x'] = snapshot[i]['x'] +\
                               snapshot[i]['speed']*time_res +\
                               snapshot[i]['accel']*time_res*time_res*0.5
            snapshot[i]['speed'] = snapshot[i]['speed'] +\
                                   snapshot[i]['accel']*time_res

        # Update the state and speed of the ego.
        # It's a bit tricky since the ego is following the lane changing path.
        ego_distance_on_path = ego_distance_on_path +\
                               snapshot[0]['speed']*time_res +\
                               snapshot[0]['accel']*time_res*time_res*0.5

        # If the ego has reached or exceeded the end of the path, stop the simulation.
        if ego_distance_on_path >= ego_path[-1]['s']: break

        ego_waypoint = interpolate_path(ego_path, ego_distance_on_path)
        snapshot[0]['x'] = ego_waypoint['x']
        snapshot[0]['y'] = ego_waypoint['y']
        snapshot[0]['theta'] = ego_waypoint['theta']
        snapshot[0]['speed'] = snapshot[0]['speed'] + snapshot[0]['accel']*time_res

        if check_collision(snapshot):
            print('Collision snapshot: \n', snapshot)
            raise RuntimeError('Collision detected during the simulation.')

if __name__ == '__main__':
    main()
