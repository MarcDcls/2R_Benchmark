import json
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal as signal
import placo
import placo_utils.tf as tf
from sklearn.linear_model import LinearRegression
from control_R1 import dfsin, ddfsin, fsin, FRAMERATE


def apply_butter_lowpass(data, fs, cutoff=50, order=4):
    """ 
    Apply a Butterworth lowpass filter.
    data: the data to be filtered
    fs: the sampling frequency of the data
    cutoff: the cutoff frequency of the filter (Hz)
    order: the order of the filter (multiples of 2)
    """
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq

    b, a = signal.butter(order, normal_cutoff, btype='low', analog=False)
    return signal.lfilter(b, a, data)

def derivative(data, timestamps):
    """ Compute the derivative of the data """
    derivative = []
    for i in range(1, len(data) - 1):
        derivative.append((data[i + 1] - data[i - 1]) / (timestamps[i + 1] - timestamps[i - 1]))
    return derivative

def plot_ts(filename):
    """ Plot the timestamps (use to check if the framerate id constant) """
    with open(filename, 'r') as f:
        data = json.load(f)

    timestamps = np.diff(data["timestamps"])
    plt.plot(timestamps)
    plt.axhline(1/FRAMERATE, color="red", label="goal_framerate")
    plt.show()

def plot_sinus(filename="logs_R1/R1_sinus_0w.json"):
    """ Plot the read and goal position and velocity """
    with open(filename, 'r') as f:
        data = json.load(f)

    plt.plot(data["timestamps"], data["read_position"], label="read position")
    plt.plot(data["timestamps"], data["goal_position"], label="goal position")
    plt.legend()
    plt.show()

    plt.plot(data["timestamps"], data["read_velocity"], label="read velocity")  
    plt.plot(data["timestamps"], data["goal_velocity"], label="goal velocity")
    plt.legend()
    plt.show()

def alternative_torque_cos(theta):
    """ Estimate the torque from the position """
    m = .35
    l = .17
    g = 9.81
    return m * g * l * np.cos(theta)

def alternative_torque_sin(theta):
    """ Estimate the torque from the position """
    m = .35
    l = .17
    g = 9.81
    return m * g * l * np.sin(theta)

def plot_estimated_velocity(filename):
    """ Plot the read and estimated_velocities """
    with open(filename, 'r') as f:
            data = json.load(f)

    timestamps = data["timestamps"]
    position = data["read_position"]
    velocity = data["read_velocity"]

    estimated_velocity = derivative(position, timestamps)
    plt.plot(timestamps[1:-1], estimated_velocity, label="estimated_velocity")
    plt.plot(timestamps, velocity, label="read_velocity")
    plt.xlabel("time [s]")
    plt.ylabel("Velocity [rad/s]")
    plt.legend()
    plt.show()

def plot_torque_and_current(filename, y_rotation, show_data=True, false_torque="None", configuration="default"):
    """ Plot the estimated torque and the read current """
    # Load the robot
    if configuration == "default":
        robot = placo.RobotWrapper("models/1R_arm/", placo.Flags.ignore_collisions)
    else:
        robot = placo.RobotWrapper("models/1R_arm_" + configuration, placo.Flags.ignore_collisions)
    robot.set_joint("R1", 1e-5)
    robot.update_kinematics()
    solver = robot.make_solver()

    # Placing the base in correct orientation
    T_world_base = robot.get_T_world_frame("base")
    T_world_base = T_world_base @ tf.tf.rotation_matrix(y_rotation, np.array([0., 1., 0.]))
    solver.add_frame_task("base", T_world_base)
    for i in range(10):
        solver.solve(True)
        robot.update_kinematics()

    with open(filename, 'r') as f:
            data = json.load(f)

    timestamps = data["timestamps"]
    position = data["read_position"]
    velocity = data["read_velocity"]
    current = data["read_current"]
    pwm = data["read_pwm"]

    estimated_acceleration = derivative(velocity, timestamps)
    estimated_torque = []
    for i in range(1, len(timestamps) - 1):
        
        robot.set_joint("R1", position[i])
        solver.solve(True)
        robot.update_kinematics()
        if false_torque == "cos":
            estimated_torque.append(alternative_torque_cos(position[i]))
        elif false_torque == "sin":
            estimated_torque.append(alternative_torque_sin(position[i]))
        else:
            estimated_torque.append(robot.torques_from_acceleration_with_fixed_frame_dict(np.array([estimated_acceleration[i - 1]]), "base")["R1"])

    if show_data:
        fig, ax1 = plt.subplots()
        ax1.plot(timestamps, position, label="read_position", color="blue")
        ax1.set_xlabel("time [s]")
        # ax1.set_ylabel("Position [rad]")

        ax2 = ax1.twinx()
        ax2.plot(timestamps, velocity, label="read_velocity", color="red")
        # ax2.set_ylabel("Velocity [rad/s]")

        ax3 = ax1.twinx()
        ax3.plot(timestamps[1:-1], estimated_acceleration, label="estimated_acceleration", color="green")
        # ax3.set_ylabel("Acceleration [rad/s^2]")

        ax4 = ax1.twinx()
        ax4.plot(timestamps, current, label="read_current", color="pink")
        # ax4.set_ylabel("Current [mA]")

        ax5 = ax1.twinx()
        ax5.plot(timestamps[1:-1], estimated_torque, label="estimated_torque", color="orange")
        # ax5.set_ylabel("Torque [Nm]")

        ax6 = ax1.twinx()
        ax6.plot(timestamps, pwm, label="read_pwm", color="purple")
        # ax6.set_ylabel("PWM [%]")

        if "goal_current" in data:
            goal_current = data["goal_current"]
            axgc = ax1.twinx()
            axgc.plot(timestamps, goal_current, label="goal_current")

        if "goal_pwm" in data:
            goal_pwm = data["goal_pwm"]
            axgp = ax1.twinx()
            axgp.plot(timestamps, goal_pwm, label="goal_pwm")

        fig.legend()
        plt.title("Data from %s" % filename)
        plt.show()

    # Add a gradient to the plot
    gradient_variable = []
    for d in np.diff(position[1:]):
        gradient_variable.append(d * 1000)    
    
    # Fit a linear regression to the data
    X = np.array(estimated_torque).reshape(-1, 1)
    y = np.array(current[1:-1]).reshape(-1, 1)
    reg = LinearRegression(fit_intercept=False).fit(X, y)
    print(f"Linear regression coefficients: {reg.coef_[0][0]/1000} A/Nm")

    plt.scatter(estimated_torque, current[1:-1], label="current", c=gradient_variable, cmap='viridis', s=10)
    plt.plot(X, reg.predict(X), color='red', linewidth=2)
    plt.ylabel("Current [mA]")
    plt.xlabel("Torque [Nm]")
    plt.legend()
    plt.title("Current from Torque (linear regression)")
    plt.show()
    
    # Filtering the data to remove the high current peaks
    # def filter_current(mx_diff=30):
    #     filtered_current = []
    #     filtered_torque = []
    #     diff_current = np.diff(current)
    #     max_diff = 30 # mA
    #     for i in range(1, len(timestamps) - 1):
    #         if abs(diff_current[i]) < max_diff:
    #             filtered_current.append(current[i])
    #             filtered_torque.append(estimated_torque[i - 1])
    #     plt.scatter(filtered_torque, filtered_current, label="current", s=10)
    #     plt.ylabel("Current [mA]")
    #     plt.xlabel("Torque [Nm]")
    #     plt.legend()
    #     plt.title("Current from Torque (filtered)")
    #     plt.show()
    
    # filter_current(5)
    # filter_current(1)
    # filter_current(0.5)
    # filter_current(0.1)

def plot_current_and_velocity(filename):
    """ Plot the read current and velocity """
    with open(filename, 'r') as f:
            data = json.load(f)

    timestamps = data["timestamps"]
    velocity = data["read_velocity"]
    current = data["read_current"]

    fig, ax1 = plt.subplots()
    ax1.plot(timestamps, current, label="read_current", color="red")
    ax1.set_ylabel("Current [mA]")
    ax1.set_xlabel("Time [s]")
    ax2 = ax1.twinx()
    ax2.plot(timestamps, velocity, label="read_velocity", color="blue")
    ax2.set_ylabel("Velocity [rad/s]")
    fig.legend()
    plt.title("Read current and velocity")
    plt.show()

def plot_velocity_vs_torque(filename, y_rotation, configuration="default"):
    # Load the robot
    if configuration == "default":
        robot = placo.RobotWrapper("models/1R_arm/", placo.Flags.ignore_collisions)
    else:
        robot = placo.RobotWrapper("models/1R_arm_" + configuration, placo.Flags.ignore_collisions)
    robot.set_joint("R1", 1e-5)
    robot.update_kinematics()
    solver = robot.make_solver()

    # Placing the base in correct orientation
    T_world_base = robot.get_T_world_frame("base")
    T_world_base = T_world_base @ tf.tf.rotation_matrix(y_rotation, np.array([0., 1., 0.]))
    solver.add_frame_task("base", T_world_base)
    for i in range(10):
        solver.solve(True)
        robot.update_kinematics()

    with open(filename, 'r') as f:
            data = json.load(f)

    timestamps = data["timestamps"]
    position = data["read_position"]
    velocity = data["read_velocity"]

    estimated_acceleration = derivative(velocity, timestamps)
    estimated_torque = []
    for i in range(1, len(timestamps) - 1):
        robot.set_joint("R1", position[i])
        solver.solve(True)
        robot.update_kinematics()
        estimated_torque.append(robot.torques_from_acceleration_with_fixed_frame_dict(np.array([estimated_acceleration[i - 1]]), "base")["R1"])

    plt.scatter(estimated_torque, velocity[1:-1], label="velocity", s=3)
    plt.ylabel("Velocity [rad/s]")
    plt.xlabel("Torque [Nm]")
    plt.legend()
    plt.title("Velocity from Torque")
    plt.show()


if __name__ == '__main__':
    
    # plot_velocity_vs_torque("logs/R1_random_pwm_16-15-03.json", np.pi)

    # plot_ts("logs/constant_pwm_1k.json")

    # plot_current_and_velocity("logs/constant_velocity.json")
    # plot_current_and_velocity("logs/constant_pwm_1k.json")
    # plot_current_and_velocity("logs/constant_pwm_300.json")

    # plot_estimated_velocity("logs/R1_random_pwm_22-14-10.json")

    # plot_torque_and_current("logs/R1_random_pwm_16-15-03.json", np.pi) # 300s
    # plot_torque_and_current("logs/R1_random_pwm_16-15-09.json", np.pi) # 300s
    # plot_torque_and_current("logs/R1_random_pwm_22-14-10.json", np.pi) # 10s
    # plot_torque_and_current("logs/R1_random_pwm_22-14-36.json", np.pi) # 300s
    # plot_torque_and_current("logs/R1_random_pwm_22-14-30.json", np.pi) # 300s
    # plot_torque_and_current("logs/R1_random_pwm_22-14-29.json", np.pi) # 10s
    # plot_torque_and_current("logs/R1_random_pwm_22-14-28.json", np.pi) # 10s

    ########### ROUGHLY ESTIMATED COEFFICIENT ###########
    #
    #                 Ki = 0.48 A/Nm
    #
    #####################################################

    # plot_torque_and_current("logs/R1_pwm_30.json", np.pi)
    # plot_torque_and_current("logs/R1_pwm_20.json", np.pi)
    # plot_torque_and_current("logs/R1_pwm_15.json", np.pi)
    # plot_torque_and_current("logs/R1_pwm_10.json", np.pi)

    # plot_torque_and_current("logs/R1_current_3.json", np.pi)
    # plot_torque_and_current("logs/R1_current_2.json", np.pi)
    # plot_torque_and_current("logs/R1_current_1.json", np.pi)

    # plot_torque_and_current("logs/R1_sinus.json", np.pi/2)

    # for i in {0, 8}:
    #     plot_sinus(f"logs_1R/R1_sinus_{i}w.json")

    plot_sinus(f"logs_1R/R1_var_sinus_0w.json")
    plot_sinus(f"logs_1R/R1_var_sinus_2w.json")
    plot_sinus(f"logs_1R/R1_var_sinus_4w.json")