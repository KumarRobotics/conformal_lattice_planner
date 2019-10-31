#!/usr/bin/python3

#from __future__ import print_function
#from __future__ import division
from math import sqrt
from math import tanh

time_gap = 1.0
distance_gap = 6.0

accel_exp = 4.0

comfort_accel = 1.5
comfort_decel = 2.5

max_accel = 5.0
max_decel = 8.0

coolness_factor = 0.9

def saturate_accel(accel):

    if accel > max_accel:
        accel = max_accel
    if accel < -max_decel:
        accel = -max_decel

    return accel

def desired_distance(ego_v, lead_v):
    v_diff = ego_v - lead_v
    return distance_gap + max(
            0.0,
            ego_v*time_gap + ego_v*v_diff / (2.0*sqrt(comfort_accel*comfort_decel)))

def intelligent_driver_model(ego_v, ego_v0, lead_v=None, s=None):

    if (lead_v is None) or (s is None):
        accel_out = comfort_accel * (1.0 - (ego_v/ego_v0)**accel_exp)
    else:
        accel_out = comfort_accel * (1.0 -
                    (ego_v/ego_v0)**accel_exp -
                    (desired_distance(ego_v, lead_v)/s)**2.0)

    return saturate_accel(accel_out)

def free_accel(ego_v, ego_v0):

    if ego_v <= ego_v0:
        return comfort_accel * (
                1.0-(ego_v/ego_v0)**accel_exp)
    else:
        return -comfort_decel * (
                1.0-(ego_v0/ego_v)**(comfort_accel*accel_exp/comfort_decel))

def improved_intelligent_driver_model(ego_v, ego_v0, lead_v=None, s=None):

    accel_free = free_accel(ego_v, ego_v0)

    if (lead_v is None) or (s is None):
        return accel_free

    z = desired_distance(ego_v, lead_v) / s

    if ego_v<ego_v0 and z>=1:
        accel_out = comfort_accel * (1.0-z**2)
    elif ego_v<ego_v0 and z<1:
        accel_out = accel_free * (1.0-z**(2.0*comfort_accel/accel_free))
    elif ego_v>=ego_v0 and z>=1:
        accel_out = accel_free + comfort_accel*(1-z**2.0)
    else:
        accel_out = accel_free

    return saturate_accel(accel_out)

def const_accel_heuristic(ego_v, lead_v, lead_v_dot, s):

    accel_tilde = min(comfort_accel, lead_v_dot)

    if lead_v*(ego_v-lead_v) < -2.0*s*accel_tilde:
        return (ego_v**2.0 * accel_tilde) / (
                lead_v**2.0 - 2.0*s*accel_tilde)
    else:
        Theta = lambda x : 1 if x>=0 else 0
        return accel_tilde - (ego_v-lead_v)**2.0*Theta(ego_v-lead_v) / (2.0*s)

def adaptive_cruise_control(ego_v, ego_v0, lead_v=None, s=None):

    lead_v_dot = 0.0
    accel_iidm = improved_intelligent_driver_model(ego_v, ego_v0, lead_v, s)

    if (lead_v is None) or (s is None):
        return accel_iidm

    accel_cah = const_accel_heuristic(ego_v, lead_v, lead_v_dot, s)

    if accel_iidm >= accel_cah:
        accel_out = accel_iidm
    else:
        accel_out = (1-coolness_factor) * accel_iidm + coolness_factor * (
                    accel_cah + comfort_decel*tanh((accel_iidm-accel_cah)/comfort_decel))
    return saturate_accel(accel_out)

def main():

    ego_v  = 10
    ego_v0 = 29

    print("ego v: ", ego_v, "ego v0: ", ego_v0)
    print("IDM: ", intelligent_driver_model(ego_v, ego_v0))
    print("IIDM: ", improved_intelligent_driver_model(ego_v, ego_v0))
    print("ACC: ", adaptive_cruise_control(ego_v, ego_v0))
    print("")

    ego_v  = 40
    ego_v0 = 29

    print("ego v: ", ego_v, "ego v0: ", ego_v0)
    print("IDM: ", intelligent_driver_model(ego_v, ego_v0))
    print("IIDM: ", improved_intelligent_driver_model(ego_v, ego_v0))
    print("ACC: ", adaptive_cruise_control(ego_v, ego_v0))
    print("")

    ego_v = 20
    ego_v0 = 29
    lead_v = 29
    s = 50
    print("ego v: ", ego_v, "ego v0: ", ego_v0, "lead v: ", lead_v, "s: ", s)
    print("IDM: ", intelligent_driver_model(ego_v, ego_v0, lead_v, s))
    print("IIDM: ", improved_intelligent_driver_model(ego_v, ego_v0, lead_v, s))
    print("ACC: ", adaptive_cruise_control(ego_v, ego_v0, lead_v, s))
    print("")

    ego_v = 25
    ego_v0 = 29
    lead_v = 15
    s = 50
    print("ego v: ", ego_v, "ego v0: ", ego_v0, "lead v: ", lead_v, "s: ", s)
    print("IDM: ", intelligent_driver_model(ego_v, ego_v0, lead_v, s))
    print("IIDM: ", improved_intelligent_driver_model(ego_v, ego_v0, lead_v, s))
    print("ACC: ", adaptive_cruise_control(ego_v, ego_v0, lead_v, s))
    print("")

    ego_v = 29
    ego_v0 = 29
    lead_v = 0
    s = 100
    print("ego v: ", ego_v, "ego v0: ", ego_v0, "lead v: ", lead_v, "s: ", s)
    print("IDM: ", intelligent_driver_model(ego_v, ego_v0, lead_v, s))
    print("IIDM: ", improved_intelligent_driver_model(ego_v, ego_v0, lead_v, s))
    print("ACC: ", adaptive_cruise_control(ego_v, ego_v0, lead_v, s))
    print("")

if __name__ == '__main__':
    main()


