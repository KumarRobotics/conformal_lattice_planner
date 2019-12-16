#!/usr/bin/env python

from __future__ import division
from __future__ import print_function

import numpy as np
import matplotlib.pyplot as plt

import rosbag
from conformal_lattice_planner.msg import EgoPlanGoal, EgoPlanResult
from conformal_lattice_planner.msg import AgentPlanGoal, AgentPlanResult

def main():

    ego_data_type = np.dtype([
        ('t', 'float'),
        ('speed', 'float'),
        ('acceleration', 'float'),
        ('leading_distance', 'float'),
        ('following_distance', 'float'),
        ('policy_speed', 'float'),
        ('path_type', 'int'),
        ('planning_time', 'float')])

    bagfile = '/home/ke/.ros/traffic_data_2019-12-16-15-57-15.bag'
    bag = rosbag.Bag(bagfile, 'r')

    ego_data = np.zeros(
            bag.get_message_count('/carla/carla_simulator/ego_plan/goal'),
            dtype=ego_data_type)
    ego_data_counter = 0

    for topic, msg, t in bag.read_messages(topics=[
        '/carla/carla_simulator/ego_plan/goal',
        '/carla/carla_simulator/ego_plan/result']):

        if topic=='/carla/carla_simulator/ego_plan/goal':
            ego_data[ego_data_counter]['t'] = msg.goal.simulation_time
            ego_data[ego_data_counter]['speed'] = msg.goal.snapshot.ego.speed
            ego_data[ego_data_counter]['acceleration'] = msg.goal.snapshot.ego.acceleration
            ego_data[ego_data_counter]['leading_distance'] = msg.goal.leading_distance
            ego_data[ego_data_counter]['following_distance'] = msg.goal.following_distance
        else:
            ego_data[ego_data_counter]['path_type'] = msg.result.path_type
            ego_data[ego_data_counter]['planning_time'] = msg.result.planning_time
            ego_data_counter += 1

    bag.close()

    fig, ax = plt.subplots()
    ax.plot(ego_data['t'], ego_data['path_type'])
    ax.grid()
    ax.set_xlabel('time (s)')
    #ax.set_ylabel('speed (m/s)')
    plt.show()

if __name__ == '__main__':
    main()
