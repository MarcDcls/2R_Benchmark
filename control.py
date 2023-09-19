import time
import numpy as np
import matplotlib.pyplot as plt
from trajectory import get_motors_positions, get_motors_position, initial_configuration
from motor import *
import argparse


def R1_sinus(id, read_pos=True, read_current=True, duration=5):
    """ Sinusoidal trajectory on a R1 arm with read position and read current plots """

    # Position Control Mode
    disable_torque()
    set_control_mode(POSITION_CONTROL_MODE)
    enable_torque()

    # Reaching initial position
    set_position(id, 0)
    time.sleep(1)

    # Trajectory
    read_positions = []
    read_currents = []
    t0 = time.time()
    t = time.time() - t0
    while t < duration:
        set_position(id, np.pi/2 * np.sin(t * np.pi/2))
        if read_pos:
            read_positions.append((t, get_position(id)))
        if read_current:
            read_currents.append((t, get_current(id)))
        t = time.time() - t0

    # Freeing the motor
    set_position(2, np.pi/2, write_only=True)

    # Reference values
    goal_positions = []
    for t in np.arange(0, duration, 1e-5):
        goal_positions.append((t, np.pi/2 * np.sin(t * np.pi/2)))
    goal_positions = np.array(goal_positions)

    # Plotting
    if read_pos:
        read_positions = np.array(read_positions)
        plt.plot(read_positions.T[0], read_positions.T[1], label="read_position")
        plt.plot(goal_positions.T[0], goal_positions.T[1], label="goal_position")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (rad)")
        plt.legend()
        plt.show()

    if read_current:
        read_currents = np.array(read_currents)
        fig, ax1 = plt.subplots()
        ax1.plot(read_currents.T[0], read_currents.T[1], label="read_current")
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Position (rad)")
        ax2 = ax1.twinx()
        ax2.plot(goal_positions.T[0], goal_positions.T[1], label="goal_position", color="red")
        ax2.set_ylabel("Current (mA)")
        fig.legend()
        plt.show()        

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

    # R1_sinus(2)
    # current_threshold_identification(1)

    # Current Control Mode
    disable_torque()
    set_control_mode(CURRENT_CONTROL_MODE)
    enable_torque()

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
