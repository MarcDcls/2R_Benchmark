import time
import numpy as np
import matplotlib.pyplot as plt
from trajectory import get_motors_positions, get_motors_position, initial_configuration
from motor import *
import argparse


def measure_getting_time(n=300):
    t0 = time.time()
    print()
    for i in range(n):
        get_position(1)
    return (time.time() - t0) / n

def measure_setting_write_only_time(n=300):
    t0 = time.time()
    for i in range(n):
        set_position(1, np.pi/2, write_only=True)
    return (time.time() - t0) / n
    
def measure_setting_read_write_time(n=300):
    t0 = time.time()
    error_count = 0
    for i in range(n):
        if not set_position(1, np.pi/2, write_only=False):
            error_count += 1
    return (time.time() - t0) / n, error_count

def reaching_initial_configuration():
    """Reaching the initial configuration of the robot."""
    print("Reaching initial configuration...")
    q = initial_configuration()
    set_position(1, q[0], write_only=False)
    set_position(2, q[1], write_only=False)
    time.sleep(1)
    print("Initial configuration reached!")

def simple_movement_plot(label):
    t0 = time.time()
    t = t0
    read_positions = []
    
    while t - t0 < args.duration:
        read_positions.append((t - t0, get_position(1), get_position(2)))
        t_test = time.time()
        r1, r2 = get_motors_position(t - t0, traj_type=args.trajectory, period=args.period)
        print(f"{time.time() - t_test} s")
        set_position(1, r1, write_only=False)
        set_position(2, r2, write_only=False)
        t = time.time()

    read_positions = np.array(read_positions)
    plt.plot(read_positions.T[0], read_positions.T[1], label=f"R1_{label}")
    plt.plot(read_positions.T[0], read_positions.T[2], label=f"R2_{label}")


if __name__ == '__main__':
    # Parsing arguments with long and short options
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", "-t", help="Trajectory type (circle or line)", default="circle")
    parser.add_argument("--duration", "-d", help="Duration of the trajectory", default=3, type=float)
    parser.add_argument("--period", "-p", help="Period of the trajectory", default=1, type=float)
    args = parser.parse_args()

    init_connection()

    set_return_status(RETURN_STATUS_ALL)
    set_return_delay_time(0)
    set_moving_threshold(10) # Default: 10

    print("Motors initialization done!")
    time.sleep(2)

    enable_torque()

    # Measuring timings
    print(f"Getting time: {measure_getting_time()} s")
    print(f"Setting time (write only): {measure_setting_write_only_time()} s")
    measured_time, error_count = measure_setting_read_write_time()
    print(f"Setting time (read/write): {measured_time} s")
    print(f"Error count: {error_count}")

    # # Position Control Mode
    # disable_torque()
    # set_control_mode([1, 2], POSITION_CONTROL_MODE)
    # enable_torque()

    # reaching_initial_configuration()
    # simple_movement_plot("position")

    # # Trajectory in position control mode   
    # disable_torque()
    # set_control_mode([1, 2], CURRENT_BASED_POSITION_CONTROL_MODE)
    # enable_torque()
    
    # reaching_initial_configuration()
    # simple_movement_plot("current_based_position")

    # Freeing the motors
    set_position(1, 90, write_only=False)
    set_position(2, 0, write_only=False)
    time.sleep(1)

    disable_torque()
    close_connection()

    # # Plotting the goal trajectory  
    # goal_positions = get_motors_positions(args.duration, traj_type=args.trajectory, period=args.period, framerate=0.001)
    # goal_positions = np.array(goal_positions)
    # timeline = np.arange(0, args.duration, 0.001)
    # plt.plot(timeline, goal_positions.T[0], label="R1_goal")
    # plt.plot(timeline, goal_positions.T[1], label="R2_goal")

    # plt.show()
