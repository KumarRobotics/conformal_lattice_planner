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

def prune_path_type(path_type):

    marker = 0
    value = path_type[0]
    min_stride_len = 20

    for i in range(path_type.size):
        if path_type[i] != value:
            if value != 0:
                stride = i - marker
                if stride < min_stride_len: path_type[marker:i] = 0
            marker = i
            value = path_type[i]

    if value != 0:
        stride = path_type.size - marker
        if stride < min_stride_len: path_type[marker:path_type.size] = 0

    return

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
    return ego_data

def main():

    data_dir = '/home/ke/Data/conformal_lattice_planner/braking_scenario/'
    idm_bagfile         = data_dir + 'idm_lane_follower/' + 'traffic_data_2020-01-03-14-44-16.bag'
    const_accel_bagfile = data_dir + 'spatiotemporal_lane_follower/' + 'traffic_data_2020-01-03-14-42-29.bag'

    idm_data         = extract_ego_data(idm_bagfile)
    const_accel_data = extract_ego_data(const_accel_bagfile)

    # Acceleration.
    accel_fig, accel_ax = plt.subplots()
    accel_ax.plot(        idm_data['t'],         idm_data['acceleration'], linewidth=2.0)
    accel_ax.plot(const_accel_data['t'], const_accel_data['acceleration'], linewidth=2.0)
    accel_ax.grid()
    accel_ax.set_xlabel('time (s)')
    accel_ax.set_ylabel('acceleration (m/s/s)')

    accel_fig.tight_layout()
    plt.savefig('/home/ke/Desktop/braking_scenario_acceleration.png',
                format='png', bbox_inches='tight')

    # Speed.
    speed_fig, speed_ax = plt.subplots()
    speed_ax.plot(        idm_data['t'],         idm_data['speed'])
    speed_ax.plot(const_accel_data['t'], const_accel_data['speed'])
    speed_ax.grid()
    speed_ax.set_xlabel('time (s)')
    speed_ax.set_ylabel('speed (m/s)')

    speed_fig.tight_layout()
    plt.savefig('/home/ke/Desktop/braking_scenario_speed.png',
                format='png', bbox_inches='tight')

    # Following distance.
    headway_fig, headway_ax = plt.subplots()
    headway_ax.plot(        idm_data['t'],         idm_data['leading_distance']/        idm_data['speed'])
    headway_ax.plot(const_accel_data['t'], const_accel_data['leading_distance']/const_accel_data['speed'])
    headway_ax.grid()
    headway_ax.set_xlabel('time (s)')
    headway_ax.set_ylabel('headway (s)')

    headway_fig.tight_layout()
    plt.savefig('/home/ke/Desktop/braking_scenario_headway.png',
                format='png', bbox_inches='tight')

    #plt.show()

if __name__ == '__main__':
    main()
