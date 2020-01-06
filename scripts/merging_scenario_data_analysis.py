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

def draw_vehicle(ax, vehicle, opacity, facecolor, edgecolor=None, zorder=10):

    p0 = np.array([-vehicle['y'], vehicle['x']])
    dp = np.array([-vehicle['length'], -vehicle['width']])

    theta = vehicle['yaw'] + math.pi/2.0
    R = np.array([[cos(theta), -sin(theta)],
                  [sin(theta),  cos(theta)]])

    p = R.dot(dp) + p0

    rect = patches.Rectangle((p[0], p[1]),
                             vehicle['length']*2,
                             vehicle['width']*2,
                             degrees(theta),
                             alpha=opacity,
                             fc=facecolor,
                             ec=edgecolor,
                             zorder=zorder)
    ax.add_patch(rect)

def main():

    data_dir = '/home/ke/Data/conformal_lattice_planner/merging_scenario/'
    bagfile  = data_dir + 'slc_lattice_planner/' + 'traffic_data_2020-01-03-14-48-17.bag'

    ego_data = extract_vehicle_data(bagfile, 213)
    agent214_data = extract_vehicle_data(bagfile, 214)
    agent215_data = extract_vehicle_data(bagfile, 215)
    agent216_data = extract_vehicle_data(bagfile, 216)

    fig, ax = plt.subplots()
    ax.plot(
        np.linspace(np.amin([-ego_data[     'y'][ 0],
                             -agent214_data['y'][ 0],
                             -agent215_data['y'][ 0],
                             -agent216_data['y'][ 0]]),
                    np.amax([-ego_data[     'y'][-1],
                             -agent214_data['y'][-1],
                             -agent215_data['y'][-1],
                             -agent216_data['y'][-1]]),
                    100),
        4.596*np.ones(100), 'k--')
    ax.plot(
        np.linspace(np.amin([-ego_data[     'y'][ 0],
                             -agent214_data['y'][ 0],
                             -agent215_data['y'][ 0],
                             -agent216_data['y'][ 0]]),
                    np.amax([-ego_data[     'y'][-1],
                             -agent214_data['y'][-1],
                             -agent215_data['y'][-1],
                             -agent216_data['y'][-1]]),
                    100),
        7.631*np.ones(100), 'k--')

    for i in range(0, ego_data.size, 13):

        draw_vehicle(ax, agent214_data[i], 0.8/agent214_data.size*i+0.2, 'C2')
        draw_vehicle(ax, agent215_data[i], 0.8/agent215_data.size*i+0.2, 'C3')
        draw_vehicle(ax, agent216_data[i], 0.8/agent216_data.size*i+0.2, 'C4')
        draw_vehicle(ax, ego_data[i], 0.8/ego_data.size*i+0.2, 'C0', 'black', 100)

    ax.set_ylim(2, 10)
    ax.set_aspect('equal')
    plt.show()

if __name__ == '__main__':
    main()
