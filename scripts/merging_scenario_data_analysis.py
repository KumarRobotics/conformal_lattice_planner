#!/usr/bin/env python

from __future__ import division
from __future__ import print_function

import os
import os.path
import math
from math import atan2, sin, cos, degrees

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import mpl_toolkits.mplot3d.art3d as art3d

import rosbag
from tf.transformations import euler_from_quaternion
from conformal_lattice_planner.msg import EgoPlanGoal, EgoPlanResult
from conformal_lattice_planner.msg import AgentPlanGoal, AgentPlanResult

def extract_vehicle_data(bagfile, vehicle_id):

    data_type = np.dtype([
        (     't', 'float'),
        (     'x', 'float'),
        (     'y', 'float'),
        (   'yaw', 'float'),
        ('length', 'float'),
        ( 'width', 'float')])

    bag = rosbag.Bag(bagfile, 'r')
    data_counter = 0
    vehicle_data = np.zeros(bag.get_message_count('/carla/carla_simulator/ego_plan/goal'), dtype=data_type)

    # Extract all data from the bag.
    for topic, msg, t in bag.read_messages(topics=[
        '/carla/carla_simulator/ego_plan/goal']):

        # Get the vehicle with the same ID.
        if vehicle_id == msg.goal.snapshot.ego.id:
            vehicle = msg.goal.snapshot.ego
        else:
            for agent in msg.goal.snapshot.agents:
                if agent.id != vehicle_id: continue
                vehicle = agent

        q = vehicle.transform.orientation
        vehicle_data[data_counter][     't'] = msg.goal.simulation_time
        vehicle_data[data_counter][     'x'] = vehicle.transform.position.x
        vehicle_data[data_counter][     'y'] = vehicle.transform.position.y
        vehicle_data[data_counter][   'yaw'] = atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))
        vehicle_data[data_counter]['length'] = vehicle.bounding_box.extent.x
        vehicle_data[data_counter][ 'width'] = vehicle.bounding_box.extent.y

        data_counter = data_counter + 1

    bag.close()
    return vehicle_data

def vehicle_path_geom(vehicle):

    x = -vehicle['y']
    y =  vehicle['x']
    return x, y

def vehicle_patch_geom(vehicle):

    p0 = np.array([-vehicle['y'], vehicle['x']])
    dp = np.array([-vehicle['length'], -vehicle['width']])

    theta = vehicle['yaw'] + math.pi/2.0
    R = np.array([[cos(theta), -sin(theta)],
                  [sin(theta),  cos(theta)]])

    p = R.dot(dp) + p0

    return p, degrees(theta)

def main():

    data_dir = '/home/ke/Data/conformal_lattice_planner/merging_scenario/'
    bagfile  = data_dir + 'slc_lattice_planner/' + 'traffic_data_2020-01-03-14-48-17.bag'

    ego_data = extract_vehicle_data(bagfile, 213)
    agent214_data = extract_vehicle_data(bagfile, 214)
    agent215_data = extract_vehicle_data(bagfile, 215)
    agent216_data = extract_vehicle_data(bagfile, 216)

    time_instances = [0, 30, 40, 65]
    figs = []

    #x_min = np.amin([-ego_data['y'][ 0], -agent214_data['y'][ 0], -agent215_data['y'][ 0], -agent216_data['y'][ 0]])
    #x_max = np.amax([-ego_data['y'][-1], -agent214_data['y'][-1], -agent215_data['y'][-1], -agent216_data['y'][-1]])
    x_min = 65
    x_max = 180

    for i in time_instances:

        fig, ax = plt.subplots()
        ax.plot(np.linspace(x_min, x_max, 100), 4.596*np.ones(100), 'k--', alpha=0.5)
        ax.plot(np.linspace(x_min, x_max, 100), 7.631*np.ones(100), 'k--', alpha=0.5)

        # Draw the agent 214.
        agent214_p, agent214_theta = vehicle_patch_geom(agent214_data[i])
        agent214_rect = patches.Rectangle(
                agent214_p, agent214_data['length'][i]*2, agent214_data['width'][i]*2, agent214_theta,
                alpha=1.0, fc='C7', ec='black', zorder=100)
        ax.add_patch(agent214_rect)

        agent214_x, agent214_y = vehicle_path_geom(agent214_data[0:i+1])
        for j in range(0, i-1):
            ax.plot(agent214_x[j:j+2], agent214_y[j:j+2], color='C7', alpha=0.9*j/i+0.1, linewidth=2)

        # Draw the agent 215.
        agent215_p, agent215_theta = vehicle_patch_geom(agent215_data[i])
        agent215_rect = patches.Rectangle(
                agent215_p, agent215_data['length'][i]*2, agent215_data['width'][i]*2, agent215_theta,
                alpha=1.0, fc='C7', ec='black', zorder=100)
        ax.add_patch(agent215_rect)

        agent215_x, agent215_y = vehicle_path_geom(agent215_data[0:i+1])
        for j in range(0, i-1):
            ax.plot(agent215_x[j:j+2], agent215_y[j:j+2], color='C7', alpha=0.9*j/i+0.1, linewidth=2)

        # Draw the agent 216.
        agent216_p, agent216_theta = vehicle_patch_geom(agent216_data[i])
        agent216_rect = patches.Rectangle(
                agent216_p, agent216_data['length'][i]*2, agent216_data['width'][i]*2, agent216_theta,
                alpha=1.0, fc='C7', ec='black', zorder=100)
        ax.add_patch(agent216_rect)

        agent216_x, agent216_y = vehicle_path_geom(agent216_data[0:i+1])
        for j in range(0, i-1):
            ax.plot(agent216_x[j:j+2], agent216_y[j:j+2], color='C7', alpha=0.9*j/i+0.1, linewidth=2)

        # Draw the ego vehicle.
        ego_p, ego_theta = vehicle_patch_geom(ego_data[i])
        ego_rect = patches.Rectangle(
                ego_p, ego_data['length'][i]*2, ego_data['width'][i]*2, ego_theta,
                alpha=1.0, fc='C0', ec='black', zorder=100)
        ax.add_patch(ego_rect)

        ego_x, ego_y = vehicle_path_geom(ego_data[0:i+1])
        for j in range(0, i-1):
            ax.plot(ego_x[j:j+2], ego_y[j:j+2], color='C0', alpha=0.9*j/i+0.1, linewidth=2)

        ax.set_ylim(2, 10)
        ax.set_xlim(65, 180)
        ax.set_aspect('equal')
        figs.append(fig)

        fig.tight_layout()
        plt.savefig('/home/ke/Desktop/merging_scenario_snapshot_'+str(i)+'.png',
                    format='png', bbox_inches='tight')

    plt.show()

if __name__ == '__main__':
    main()
