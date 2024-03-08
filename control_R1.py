import time
import numpy as np
import matplotlib.pyplot as plt
import json
from motor import *
import argparse

FRAMERATE = 300 # Hz
# DXL_IDS = [2]

# Sinusoidal trajectories
def alt_fsin(t):
    return 1*np.pi/3 * np.sin(t**2)

# Sinusoidal trajectories
def alt_dfsin(t):
    return 2*t*np.pi/3 * np.cos(t**2)

# Sinusoidal trajectories
def alt_ddfsin(t):
    return 2*np.pi/3 * np.cos(t**2) - 4*t**2*np.pi/3 * np.sin(t**2)

# Sinusoidal trajectories
def fsin(t):
    return 1*np.pi/3 * np.sin(t * np.pi/2)

def dfsin(t):
    return np.pi**2/6 * np.cos(t * np.pi/2)

def ddfsin(t):
    return - np.pi**3/12 * np.sin(t * np.pi/2)
    
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

def R1_position_sinus(id, duration=8, use_alt=False, filename="None"):
    """ Sinusoidal trajectory on a R1 arm using position control with read position and read current plots """
    # Position Control Mode
    disable_torque()
    set_control_mode(POSITION_CONTROL_MODE)
    enable_torque()

    # Reaching initial position
    set_position(id, 0)
    time.sleep(1)

    # Trajectory
    read_position = []
    read_velocity = []
    read_current = []
    timeline = []
    t0 = time.time()
    t_sum = - 1/FRAMERATE
    while t_sum < duration:
        t = time.time() - t0
        if t < 1/FRAMERATE:
            continue

        data = read_data(id, position=True, velocity=True, current=True, pwm=False)
        read_position.append(data["position"])
        read_velocity.append(data["velocity"])
        read_current.append(data["current"])

        t_sum += t
        if use_alt:
            set_position(id, alt_fsin(t_sum))
        else:
            set_position(id, fsin(t_sum))
        timeline.append(t_sum)
        t0 += t

    # Freeing the motor
    set_position(2, 0, write_only=True)
    time.sleep(1)
    disable_torque()

    # Reference values
    goal_position = []
    goal_velocity = []
    goal_acceleration = []
    for t in timeline:
        if use_alt:
            goal_position.append(alt_fsin(t))
            goal_velocity.append(alt_dfsin(t))
            goal_acceleration.append(alt_ddfsin(t))
        else:
            goal_position.append(fsin(t))
            goal_velocity.append(dfsin(t))
            goal_acceleration.append(ddfsin(t))

    # Saving data
    if filename != "None":
        data = {
            "timestamps": timeline,
            "read_position": read_position,
            "read_velocity": read_velocity,
            "read_current": read_current,
            "goal_position": goal_position,
            "goal_velocity": goal_velocity,
            "goal_acceleration": goal_acceleration
        }
        with open(filename, 'w') as f:
            json.dump(data, f)
        print("Data saved in %s" % filename)

def R1_current_sinus(id, max_current, period, duration=8, filename="None"):
    """Sinusoidal trajectory on a R1 arm using current control - USE ONLY ON HORIZONTAL BENCH"""    
    # Reaching initial position
    disable_torque()
    set_control_mode(POSITION_CONTROL_MODE)
    enable_torque()
    set_position(id, 0)
    time.sleep(1)

    # Current Control Mode
    disable_torque()
    set_control_mode(CURRENT_CONTROL_MODE)
    enable_torque()

    # Trajectory
    read_position = []
    read_velocity = []
    read_current = []
    read_pwm = []
    timeline = []
    t0 = time.time()
    t_sum = - 1/FRAMERATE
    while t_sum < duration:
        t = time.time() - t0
        if t < 1/FRAMERATE:
            continue

        data = read_data(id, position=True, velocity=True, current=True, pwm=True)
        read_position.append(data["position"])
        read_velocity.append(data["velocity"])
        read_current.append(data["current"])
        read_pwm.append(data["pwm"])

        t_sum += t
        set_current(id, max_current * np.sin(period * t_sum))
        timeline.append(t_sum)
        t0 += t

    # Freeing the motor
    disable_torque()

    # Reference values
    goal_current = []
    for t in timeline:
        goal_current.append(max_current * np.sin(period * t))

    # Saving data
    if filename != "None":
        data = {
            "timestamps": timeline,
            "read_position": read_position,
            "read_velocity": read_velocity,
            "read_current": read_current,
            "read_pwm": read_pwm,
            "goal_current": goal_current
        }
        with open(filename, 'w') as f:
            json.dump(data, f)
        print("Data saved in %s" % filename)
    
def R1_pwm_sinus(id, max_pwm, period, duration=8, filename="None"):
    """Sinusoidal trajectory on a R1 arm using pwm control - USE ONLY ON HORIZONTAL BENCH"""    
    # Reaching initial position
    disable_torque()
    set_control_mode(POSITION_CONTROL_MODE)
    enable_torque()
    set_position(id, 0)
    time.sleep(1)

    # PWM Control Mode
    disable_torque()
    set_control_mode(PWM_CONTROL_MODE)
    enable_torque()

    # Trajectory
    read_position = []
    read_velocity = []
    read_current = []
    read_pwm = []
    timeline = []
    t0 = time.time()
    t_sum = - 1/FRAMERATE
    while t_sum < duration:
        t = time.time() - t0
        if t < 1/FRAMERATE:
            continue

        data = read_data(id, position=True, velocity=True, current=True, pwm=True)
        read_position.append(data["position"])
        read_velocity.append(data["velocity"])
        read_current.append(data["current"])
        read_pwm.append(data["pwm"])

        t_sum += t
        set_pwm(id, max_pwm * np.sin(period * t_sum))
        timeline.append(t_sum)
        t0 += t

    # Freeing the motor
    disable_torque()

    # Reference values
    goal_pwms = []
    for t in timeline:
        goal_pwms.append(max_pwm * np.sin(period * t))

    # Saving data
    if filename != "None":
        data = {
            "timestamps": timeline,
            "read_position": read_position,
            "read_velocity": read_velocity,
            "read_current": read_current,
            "read_pwm": read_pwm,
            "goal_pwm": goal_pwms
        }
        with open(filename, 'w') as f:
            json.dump(data, f)
        print("Data saved in %s" % filename)

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

def R1_random_pwm(id, duration, pwm_duration, pwm_max, filename="None"):
    """ Random trajectory on a R1 arm using pwm control - USE CAUTIOUSLY """
    # Reaching initial position
    disable_torque()
    set_control_mode(POSITION_CONTROL_MODE)
    enable_torque()
    set_position(id, 0)
    time.sleep(1)

    # PWM Control Mode
    disable_torque()
    set_control_mode(PWM_CONTROL_MODE)
    enable_torque()

    # Trajectory
    read_position = []
    read_velocity = []
    read_current = []
    read_pwm = []
    goal_pwm = []
    pwm_target = 0
    timeline = []
    t0 = time.time()
    t_sum = - 1/FRAMERATE
    t_sum_pwm = - 1/FRAMERATE
    while t_sum < duration:
        t = time.time() - t0
        if t < 1/FRAMERATE:
            continue
        
        data = read_data(id, position=True, velocity=True, current=True, pwm=True)
        read_position.append(data["position"])
        read_velocity.append(data["velocity"])
        read_current.append(data["current"])
        read_pwm.append(data["pwm"])

        t_sum += t
        t_sum_pwm += t
        if read_position[-1] > np.pi/3:
            pwm_target = np.random.uniform(-pwm_max, 0)
            set_pwm(id, pwm_target)
            t_sum_pwm = 0
        elif read_position[-1] < -np.pi/3:
            pwm_target = np.random.uniform(0, pwm_max)
            set_pwm(id, pwm_target)
            t_sum_pwm = 0
        elif t_sum_pwm > pwm_duration:
            pwm_target = np.random.uniform(-pwm_max, pwm_max)
            set_pwm(id, pwm_target)
            t_sum_pwm = 0

        goal_pwm.append(pwm_target)
        timeline.append(t_sum)
        t0 += t

    # Freeing the motor
    disable_torque()

    # Saving data
    if filename != "None":
        data = {
            "timestamps": timeline,
            "read_position": read_position,
            "read_velocity": read_velocity,
            "read_current": read_current,
            "read_pwm": read_pwm,
            "goal_pwm": goal_pwm
        }
        with open(filename, 'w') as f:
            json.dump(data, f)
        print("Data saved in %s" % filename)

def constant_pwm(id, pwms, duration=3, filename="None"):
    """ Apply a constant pwm to a motor """
    # PWM Control Mode
    disable_torque()
    set_control_mode(PWM_CONTROL_MODE)
    enable_torque()

    # Trajectory
    read_velocity = []
    read_current = []
    goal_pwm = []
    timeline = []
    t0 = time.time()
    t_sum = -1/FRAMERATE
    pwm_index = 0
    while t_sum < duration * len(pwms):
        t = time.time() - t0
        if t < 1/FRAMERATE:
            continue
        
        data = read_data(id, position=False, velocity=True, current=True, pwm=False)
        read_velocity.append(data["velocity"])
        read_current.append(data["current"])

        if pwm_index * duration < t_sum:
            pwm_index += 1
            set_pwm(id, pwms[pwm_index - 1])

        t_sum += t
        goal_pwm.append(pwms[pwm_index - 1])
        timeline.append(t_sum)
        t0 += t

    # Freeing the motor
    disable_torque()

    # Saving data
    if filename != "None":
        data = {
            "timestamps": timeline,
            "read_velocity": read_velocity,
            "read_current": read_current,
            "goal_pwm": goal_pwm
        }
        with open(filename, 'w') as f:
            json.dump(data, f)
        print("Data saved in %s" % filename)

def constant_velocity(id, velocities, duration=3, use_rpm=False, filename="None"):
    """ Apply a constant velocity to a motor """
    # Velocity Control Mode
    disable_torque()
    set_control_mode(VELOCITY_CONTROL_MODE)
    enable_torque()

    # Trajectory
    read_current = []
    read_velocity = []
    goal_velocity = []
    timeline = []
    t0 = time.time()
    t_sum = -1/FRAMERATE
    velocity_index = 0
    while t_sum < duration * len(velocities):
        t = time.time() - t0
        if t < 1/FRAMERATE:
            continue
        
        data = read_data(id, position=False, velocity=True, current=True, pwm=False)
        read_velocity.append(data["velocity"])
        read_current.append(data["current"])

        if velocity_index * duration < t_sum:
            velocity_index += 1
            set_velocity(id, velocities[velocity_index - 1], use_rpm=use_rpm)

        t_sum += t
        goal_velocity.append(velocities[velocity_index - 1])
        timeline.append(t_sum)
        t0 += t

    # Freeing the motor
    disable_torque()

    # Saving data
    if filename != "None":
        data = {
            "timestamps": timeline,
            "read_velocity": read_velocity,
            "read_current": read_current,
            "goal_velocity": goal_velocity
        }
        with open(filename, 'w') as f:
            json.dump(data, f)
        print("Data saved in %s" % filename)


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
    time.sleep(.2)


    # Plotting functions
    # x = np.linspace(0, 6, 100)
    # y = fsin(x)
    # dy = dfsin(x)
    # ddy = ddfsin(x)
    
    # plt.plot(x, y, label="y")
    # plt.plot(x, dy, label="dy")
    # plt.plot(x, ddy, label="ddy")
    # plt.legend()
    # plt.show()

    # x = np.linspace(0, 10, 1000)
    # y = alt_fsin(x)
    # dy = alt_dfsin(x)
    # ddy = alt_ddfsin(x)

    # last = y[0]
    # for i, v in enumerate(y):
    #     if v * last < 0:
    #         print(x[i])
    #     last = v

    # plt.plot(x, y, label="y")
    # plt.plot(x, dy, label="dy")
    # plt.plot(x, ddy, label="ddy")
    # plt.legend()
    # plt.show()


    # constant_velocity(1, [0.2, 0.4, 0.6, 0.8, 1, 1.2, 1.4, 1.6, 1.8, 2], filename="logs/constant_velocity.json")
    # constant_pwm(1, [5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100], filename="logs/constant_pwm.json")    

    # name = time.strftime("%d-%H-%M")
    # R1_random_pwm(2, 300, .4, 100, filename=f"logs/R1_random_pwm_{name}.json")

    # R1_pwm_sinus(2, 30, 2, duration=20, filename="R1_pwm_30.json")
    # time.sleep(1)
    # R1_pwm_sinus(2, 20, 1, duration=20, filename="R1_pwm_20.json")
    # time.sleep(1)
    # R1_pwm_sinus(2, 15, .5, duration=20, filename="R1_pwm_15.json")
    # time.sleep(1)
    # R1_pwm_sinus(2, 10, .25, duration=20, filename="R1_pwm_10.json")
    # time.sleep(1)
    
    # R1_current_sinus(2, 200, 3, duration=20, filename="R1_current_3.json")
    # time.sleep(1)
    # R1_current_sinus(2, 200, 2, duration=20, filename="R1_current_2.json")
    # time.sleep(1)
    # R1_current_sinus(2, 200, 1, duration=20, filename="R1_current_1.json")


    nb_weight = 4
    R1_position_sinus(2, 6, use_alt=True, filename="R1_var_sinus_" + str(nb_weight) + "w.json")
    # R1_position_sinus(2, 20, filename="R1_sinus_" + str(nb_weight) + "w.json")
    
    
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
