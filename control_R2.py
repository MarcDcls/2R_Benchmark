import time
import numpy as np
import json
import matplotlib.pyplot as plt
from trajectory import get_motors_positions, random_2R_trajectory
from motor import *
import argparse

FRAMERATE = 300 # Hz
# DXL_IDS = [1, 2]

def read_data(id, position=True, velocity=True, current=True, pwm=True):
    """ Read data from the motors """
    data = {"position": None, "velocity": None, "current": None, "pwm": None}
    if position:
        while type(data["position"]) != float:
            data["position"] = get_position(id)
    if velocity:
        while type(data["velocity"]) != float:
            data["velocity"] = get_velocity(id)
    if current:
        while type(data["current"]) != float:
            data["current"] = get_current(id)
    if pwm:
        while type(data["pwm"]) != float:
            data["pwm"] = get_pwm(id)
    return data

def R2_position_circle(duration=8, filename="None"):
    """ Circle trajectory on a R2 arm using position control """
    # Position Control Mode
    disable_torque()
    set_control_mode(POSITION_CONTROL_MODE)
    enable_torque()

    # Reaching initial position
    trajectory = get_motors_positions(duration, traj_type="circle", period=1, framerate=1/FRAMERATE)
    set_position(1, trajectory[0][0])
    set_position(2, trajectory[0][1])
    time.sleep(1)

    # Trajectory
    read_position_1 = []
    read_position_2 = []
    read_velocity_1 = []
    read_velocity_2 = []
    timeline = []
    i = 0
    t0 = time.time()
    t_sum = - 1/FRAMERATE
    while i < len(trajectory) - 1:
        t = time.time() - t0
        if t < 1/FRAMERATE:
            continue

        data_1 = read_data(1, position=True, velocity=True, current=False, pwm=False)
        data_2 = read_data(2, position=True, velocity=True, current=False, pwm=False)
        read_position_1.append(data_1["position"])
        read_position_2.append(data_2["position"])
        read_velocity_1.append(data_1["velocity"])
        read_velocity_2.append(data_2["velocity"])

        i += 1
        set_position(1, trajectory[i][0])
        set_position(2, trajectory[i][1])
        t_sum += t
        timeline.append(t_sum)
        t0 += t

    # Freeing the motor
    set_position(1, 0, write_only=True)
    set_position(2, 0, write_only=True)
    time.sleep(1)
    disable_torque()

    # Saving data
    if filename != "None":
        data = {
            "timestamps": timeline,
            "read_position_1": read_position_1,
            "read_position_2": read_position_2,
            "read_velocity_1": read_velocity_1,
            "read_velocity_2": read_velocity_2,
            "goal_position_1": list(trajectory[:-1, 0]),
            "goal_position_2": list(trajectory[:-1, 1])
        }
        with open(filename, 'w') as f:
            json.dump(data, f)
        print("Data saved in %s" % filename)

def R2_random_articular_space(duration=8, max_speed=5, filename="None"):
    """ Random trajectory on a R2 arm using position control. Mouvement are smooth in the articular space """
    # Position Control Mode
    disable_torque()
    set_control_mode(POSITION_CONTROL_MODE)
    enable_torque()

    # Reaching initial position
    trajectory = random_2R_trajectory(duration, max_speed, framerate=1/FRAMERATE)
    set_position(1, trajectory[0][0])
    set_position(2, trajectory[0][1])
    time.sleep(1)

    # Trajectory
    read_position_1 = []
    read_position_2 = []
    read_velocity_1 = []
    read_velocity_2 = []
    timeline = []
    i = 0
    t0 = time.time()
    t_sum = - 1/FRAMERATE
    while i < len(trajectory) - 1:
        t = time.time() - t0
        if t < 1/FRAMERATE:
            continue

        data_1 = read_data(1, position=True, velocity=True, current=False, pwm=False)
        data_2 = read_data(2, position=True, velocity=True, current=False, pwm=False)
        read_position_1.append(data_1["position"])
        read_position_2.append(data_2["position"])
        read_velocity_1.append(data_1["velocity"])
        read_velocity_2.append(data_2["velocity"])

        i += 1
        set_position(1, trajectory[i][0])
        set_position(2, trajectory[i][1])
        t_sum += t
        timeline.append(t_sum)
        t0 += t

    # Freeing the motor
    set_position(1, 0, write_only=True)
    set_position(2, 0, write_only=True)
    time.sleep(1)
    disable_torque()

    # Saving data
    if filename != "None":
        data = {
            "timestamps": timeline,
            "read_position_1": read_position_1,
            "read_position_2": read_position_2,
            "read_velocity_1": read_velocity_1,
            "read_velocity_2": read_velocity_2,
            "goal_position_1": list(trajectory[:-1, 0]),
            "goal_position_2": list(trajectory[:-1, 1])
        }
        with open(filename, 'w') as f:
            json.dump(data, f)
        print("Data saved in %s" % filename)


if __name__ == '__main__':
    # Parsing arguments with long and short options
    # parser = argparse.ArgumentParser()
    # parser.add_argument("--trajectory", "-t", help="Trajectory type (circle or line)", default="circle")
    # parser.add_argument("--duration", "-d", help="Duration of the trajectory", default=3, type=float)
    # parser.add_argument("--period", "-p", help="Period of the trajectory", default=1, type=float)
    # args = parser.parse_args()

    init_connection()

    set_return_status(RETURN_STATUS_PING_READ)
    set_return_delay_time(0)
    set_moving_threshold(10) # Default: 10
    check_latency()

    print("Motors initialization done!")
    time.sleep(.2)

    # R2_position_circle(8, filename=f"logs_2R/R2_test.json")

    name = time.strftime("%d-%H-%M")
    R2_random_articular_space(300, max_speed=3.5, filename=f"logs_2R/R2_ra_3-5_300_{name}.json")

    disable_torque()
    close_connection()
