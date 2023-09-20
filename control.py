import time
import numpy as np
import json
import matplotlib.pyplot as plt
from trajectory import get_motors_positions, get_motors_position, initial_configuration
from motor import *
import argparse

FRAMERATE = 800 # Hz


# Sinusoidal trajectories
def fsin(t):
    return np.pi/2 + np.pi/4 * (np.sin(t * np.pi/2 + np.pi/2) - 1)

def dfsin(t):
    return np.pi**2/8 * np.cos(t * np.pi/2 + np.pi/2)

def ddfsin(t):
    return - np.pi**3/16 * np.sin(t * np.pi/2 + np.pi/2)
    
def R1_position_sinus(id, read_pos=True, read_current=True, duration=8, filename="None"):
    """ Sinusoidal trajectory on a R1 arm using position control with read position and read current plots """
    # Position Control Mode
    disable_torque()
    set_control_mode(POSITION_CONTROL_MODE)
    enable_torque()

    # Reaching initial position
    set_position(id, np.pi/2)
    time.sleep(1)

    # Trajectory
    read_positions = []
    read_currents = []
    t0 = time.time()
    t_sum = - 1/FRAMERATE
    while t_sum < duration:
        t = time.time() - t0
        if t < 1/FRAMERATE:
            continue

        # Getting values
        if read_pos:
            read_positions.append(get_position(id))
        if read_current:
            read_currents.append(get_current(id))

        # Setting values
        t_sum += t
        set_position(id, fsin(t_sum))
        t0 += t

    # Freeing the motor
    set_position(2, np.pi/2, write_only=True)
    time.sleep(1)
    disable_torque()

    # Reference values
    timeline = np.arange(0, duration + 1/FRAMERATE, 1/FRAMERATE).tolist()
    goal_positions = []
    for t in timeline:
        goal_positions.append(fsin(t))

    # Saving data
    if filename != "None":
        data = {
            "timestamps": timeline,
            "read_positions": read_positions,
            "read_currents": read_currents,
            "goal_positions": goal_positions
        }
        with open(filename, 'w') as f:
            json.dump(data, f)
        print("Data saved in %s" % filename)
    
    # Plotting
    if read_pos:
        plt.plot(timeline, read_positions, label="read_position")
        plt.plot(timeline, goal_positions, label="goal_position")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (rad)")
        plt.legend()
        plt.show()

    if read_current:
        fig, ax1 = plt.subplots()
        ax1.plot(timeline, read_currents, label="read_current")
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Position (rad)")
        ax2 = ax1.twinx()
        ax2.plot(timeline, goal_positions, label="goal_position", color="red")
        ax2.set_ylabel("Current (mA)")
        fig.legend()
        plt.show()        

def R1_current_sinus(id, duration=8):
    """Sinusoidal trajectory on a R1 arm using current control"""    
    # Reaching initial position
    disable_torque()
    set_control_mode(POSITION_CONTROL_MODE)
    enable_torque()
    set_position(id, np.pi/2)
    time.sleep(1)

    # Current Control Mode
    disable_torque()
    set_control_mode(CURRENT_CONTROL_MODE)
    enable_torque()

    # Trajectory
    t0 = time.time()
    t_sum = - 1/FRAMERATE
    while t_sum < duration:
        t = time.time() - t0
        if t < 1/FRAMERATE:
            continue

        current = robot.torques_from_acceleration_with_fixed_frame(np.array([ddfsin(t_sum)]), "base")["R1"] * 1000 / 0.021

        # Setting values
        t_sum += t
        set_current(id, current)
        t0 += t

    # Freeing the motor
    disable_torque()


def current_threshold_identification(id):
    """ Minimal current to move the motor from a static position : 
    5 index = 16.8 mA (Threshold at 15.12 mA due to rounding) """

    # Current Control Mode
    disable_torque()
    set_control_mode(CURRENT_CONTROL_MODE)
    enable_torque()

    set_current(id, 15.12 + 1e-5)
    t0 = time.time()
    while time.time() - t0 < 1:
        print(get_position(id))
    
    disable_torque()

if __name__ == '__main__':
    # Parsing arguments with long and short options
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", "-t", help="Trajectory type (circle or line)", default="circle")
    parser.add_argument("--duration", "-d", help="Duration of the trajectory", default=3, type=float)
    parser.add_argument("--period", "-p", help="Period of the trajectory", default=1, type=float)
    args = parser.parse_args()

    init_connection()

    set_return_status(RETURN_STATUS_PING_READ)
    set_return_delay_time(0)
    set_moving_threshold(10) # Default: 10
    check_latency()

    print("Motors initialization done!")
    time.sleep(1)

    # R1_position_sinus(2, filename="R1_sinus.json")
    # current_threshold_identification(1)

    # Current Control Mode
    # disable_torque()
    # set_control_mode(CURRENT_CONTROL_MODE)
    # enable_torque()

    # Reaching initial current
    # set_current(1, 100)

    # t0 = time.time()
    # t = time.time() - t0
    # while t < 5:
    #     set_current(1, 15.12 + 1e-5)
    #     t = time.time() - t0
    # set_current(1, 15.12 + 1e-5)
    # t0 = time.time()
    # while time.time() - t0 < 1:
    #     print(get_position(1))
    

    disable_torque()
    close_connection()
