import json
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal as signal
import placo
import placo_utils.tf as tf
from control import dfsin, ddfsin, fsin, FRAMERATE


# Load the robot
robot = placo.RobotWrapper('1R_arm/', placo.Flags.ignore_collisions)
robot.set_joint("R1", 1e-5)
robot.update_kinematics()
solver = robot.make_solver()

# Placing the base in correct orientation
T_world_base = robot.get_T_world_frame("base")
T_world_base = T_world_base @ tf.tf.rotation_matrix(-np.pi/2, np.array([0., 1., 0.]))
solver.add_frame_task("base", T_world_base)
for i in range(10):
    solver.solve(True)
    robot.update_kinematics()


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

def plot_position():
    """ Plot the read position and the goal position, velocity and acceleration """
    with open("R1_sinus.json", 'r') as f:
        data = json.load(f)

    positions = data["read_positions"]
    plt.plot(data["timestamps"], positions, label="positions")

    # Goal position : f(t) = np.pi/2 + np.pi/4 * (np.sin(t * np.pi/2 + np.pi/2) - 1)
    # Goal velocity : df(t) = np.pi**2/8 * np.cos(t * np.pi/2 + np.pi/2)
    # Goal acceleration : ddf(t) = - np.pi**3/16 * np.sin(t * np.pi/2 + np.pi/2)
    plt.plot(data["timestamps"], fsin(np.array(data["timestamps"])), label="goal_positions")
    plt.plot(data["timestamps"], dfsin(np.array(data["timestamps"])), label="goal_velocities")
    plt.plot(data["timestamps"], ddfsin(np.array(data["timestamps"])), label="goal_accelerations")

    plt.legend()
    plt.show()

def plot_torque_and_current():
    """ Plot the estimated torque and the read current """
    with open("R1_sinus.json", 'r') as f:
            data = json.load(f)

    timestamps = data["timestamps"]
    positions = data["read_positions"]
    currents = data["read_currents"]
    fig, ax1 = plt.subplots()
    ax1.plot(data["timestamps"], currents, label="current")
    ax2 = ax1.twinx()
    ax2.plot(data["timestamps"], positions, label="position", color="red")
    ax1.set_xlabel("time [s]")
    ax1.set_ylabel("Current [mA]")
    ax2.set_ylabel("Position [rad]")
    fig.legend()
    plt.title("Read position and current")
    plt.show()

    fs = FRAMERATE
    cutoff = 10
    order = 2
    filtered_currents = apply_butter_lowpass(currents, fs, cutoff, order)
    plt.plot(data["timestamps"], currents, label="current")
    plt.plot(data["timestamps"], filtered_currents, label="filtered_current", color="red")
    plt.xlabel("time [s]")
    plt.ylabel("Current [mA]")
    plt.title("Filtered current")
    plt.legend()
    plt.show()

    torques = []
    for i in range(len(timestamps)):
        robot.set_joint("R1", positions[i])
        solver.solve(True)
        robot.update_kinematics()
        torques.append(robot.torques_from_acceleration_with_fixed_frame(np.array([ddfsin(timestamps[i])]), "base")["R1"])

    plt.plot(data["timestamps"], torques, label="torque")
    plt.xlabel("time [s]")
    plt.ylabel("Torque [Nm]")
    plt.legend()
    plt.title("Estimated torque")
    plt.show()

    plt.plot(filtered_currents, torques, label="torque")
    plt.xlabel("Current [mA]")
    plt.ylabel("Torque [Nm]")
    plt.legend()
    plt.title("Torque from current")
    plt.show()


if __name__ == '__main__':

    # plot_position()

    plot_torque_and_current()

    # with open("R1_sinus.json", 'r') as f:
    #     data = json.load(f)

    # for i in range(1):
    #     filtered_currents = apply_butter_lowpass(currents, fs, cutoff, order)
    #     plt.plot(data["timestamps"], currents, label="currents")
    #     plt.plot(data["timestamps"], filtered_currents, label="filtered_currents", color="red")
    #     plt.xlabel("time [s]")
    #     plt.ylabel("Current [mA]")
    #     plt.title("Cutoff: " + str(cutoff))
    #     plt.legend()
    #     plt.show()
    #     # cutoff /= 2
    #     order += 2