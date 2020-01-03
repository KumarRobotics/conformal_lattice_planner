#!/usr/bin/env python

from __future__ import division
from __future__ import print_function

import os
import os.path

import numpy as np
import matplotlib.pyplot as plt

import rosbag
from conformal_lattice_planner.msg import EgoPlanGoal, EgoPlanResult
from conformal_lattice_planner.msg import AgentPlanGoal, AgentPlanResult

#def prune_path_type(path_type):
#
#    marker = 0
#    value = path_type[0]
#    min_stride_len = 20
#
#    for i in range(path_type.size):
#        if path_type[i] != value:
#            if value != 0:
#                stride = i - marker
#                if stride < min_stride_len: path_type[marker:i] = 0
#            marker = i
#            value = path_type[i]
#
#    if value != 0:
#        stride = path_type.size - marker
#        if stride < min_stride_len: path_type[marker:path_type.size] = 0
#
#    return

def lane_change_num(path_type):

    lane_changes = 0
    marker = 0
    value = path_type[0]
    min_stride_len = 20

    for i in range(path_type.size):
        if path_type[i] != value:
            if value != 0:
                stride = i - marker
                if stride >= min_stride_len: lane_changes = lane_changes + 1
            marker = i
            value = path_type[i]

    if value != 0:
        stride = path_type.size - marker
        if stride >= min_stride_len: lane_changes = lane_changes + 1

    return lane_changes

def extract_ego_data(bagfile):

    data_type = np.dtype([
        (                 't', 'float'),
        (             'speed', 'float'),
        (      'acceleration', 'float'),
        (  'leading_distance', 'float'),
        ('following_distance', 'float'),
        (      'policy_speed', 'float'),
        (         'path_type', 'int'  ),
        (     'planning_time', 'float')])

    bag = rosbag.Bag(bagfile, 'r')
    ego_data_counter = 0
    ego_data = np.zeros(bag.get_message_count('/carla/carla_simulator/ego_plan/goal'), dtype=data_type)

    # Extract all data from the bag.
    for topic, msg, t in bag.read_messages(topics=[
        '/carla/carla_simulator/ego_plan/goal',
        '/carla/carla_simulator/ego_plan/result']):

        if topic=='/carla/carla_simulator/ego_plan/goal':
            ego_data[ego_data_counter][                 't'] = msg.goal.simulation_time
            ego_data[ego_data_counter][             'speed'] = msg.goal.snapshot.ego.speed
            ego_data[ego_data_counter][      'acceleration'] = msg.goal.snapshot.ego.acceleration
            ego_data[ego_data_counter][  'leading_distance'] = msg.goal.leading_distance
            ego_data[ego_data_counter]['following_distance'] = msg.goal.following_distance
        else:
            ego_data[ego_data_counter][         'path_type'] = msg.result.path_type
            ego_data[ego_data_counter][     'planning_time'] = msg.result.planning_time
            ego_data_counter += 1

    bag.close()

    # Compute the variance of the acceleration data.
    accel_var = np.zeros(ego_data['acceleration'].size)
    for i in range(ego_data['acceleration'].size):
        l = i-49 if i-49>=0 else 0
        r = i+50 if i+50<=accel_var.size else accel_var.size
        accel_var[i] = np.var(ego_data['acceleration'][l:r])

    # Compute the average headway.
    valid_idx = np.nonzero(ego_data['leading_distance'] > 0.0)
    headway = ego_data['leading_distance'][valid_idx] / ego_data['speed'][valid_idx]

    # Return the tuple of
    # duration of the bag, speed, acceleration, leading headway, lane changes, planning time (all in average).
    return {               'simulation time': ego_data['t'][-1],
                             'average speed': np.mean(ego_data['speed']),
                      'average acceleration': np.mean(ego_data['acceleration']),
             'average acceleration variance': np.mean(accel_var),
                           'average headway': np.mean(headway),
                              'lane changes': lane_change_num(ego_data['path_type']),
                     'average planning time': np.mean(ego_data['planning_time']),
                                     'count': ego_data.size}

def main():

    data_dir = '/home/ke/Data/conformal_lattice_planner/random_traffic/'
    method = 'spatiotemporal_lattice_planner/'
    bagfiles = [data_dir+method+f for f in os.listdir(data_dir+method)
                  if os.path.isfile(os.path.join(data_dir+method, f))]

    average_data = {       'simulation time': 0.0,
                             'average speed': 0.0,
                      'average acceleration': 0.0,
             'average acceleration variance': 0.0,
                           'average headway': 0.0,
                              'lane changes': 0.0,
                     'average planning time': 0.0,
                                     'count': 0}

    for bag in bagfiles:

        print(bag)
        ego_data = extract_ego_data(bag)
        for key, value in ego_data.items():
            print(key, ': ', value)

        average_data[              'simulation time'] += ego_data[              'simulation time']
        average_data[                'average speed'] += ego_data[                'average speed'] * ego_data['count']
        average_data[         'average acceleration'] += ego_data[         'average acceleration'] * ego_data['count']
        average_data['average acceleration variance'] += ego_data['average acceleration variance'] * ego_data['count']
        average_data[              'average headway'] += ego_data[              'average headway'] * ego_data['count']
        average_data[                 'lane changes'] += ego_data[                 'lane changes']
        average_data[        'average planning time'] += ego_data[        'average planning time'] * ego_data['count']
        average_data[                        'count'] += ego_data[                        'count']

    average_data[                'average speed'] /= average_data['count']
    average_data[         'average acceleration'] /= average_data['count']
    average_data['average acceleration variance'] /= average_data['count']
    average_data[              'average headway'] /= average_data['count']
    average_data[        'average planning time'] /= average_data['count']

    print()
    for key, value in average_data.items():
        print(key, ': ', value)

    #fig, ax = plt.subplots()
    #ax.plot(ego_data['t'], ego_data['acceleration'])
    #ax.grid()
    #ax.set_xlabel('time (s)')
    ##ax.set_ylabel('speed (m/s)')
    #plt.show()

if __name__ == '__main__':
    main()
