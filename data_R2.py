import json
import matplotlib.pyplot as plt
import numpy as np
import placo
import sys
import os
import placo_utils.tf as tf
from log_processing import LogData, SAMPLE_RATE
from polyfit import spline

# Spline fitting parameters
WINDOW_SIZE = 38
DEGREE = 2
INTERSECTED_VALUES = 36

def preview_processing(src, nb_points=1000):
    """ Check spline fitting parameters """
    with open(src, 'r') as f:
        data = json.load(f)
    
    raw_timestamps = data["timestamps"][:nb_points]
    timestamps = list(np.arange(raw_timestamps[0], raw_timestamps[-1], 1 / SAMPLE_RATE))

    # Spline fitting
    R1_read_spline = spline(window_size=WINDOW_SIZE, degree=DEGREE, intersected_values=INTERSECTED_VALUES, x=raw_timestamps, y=data["read_position_1"][:nb_points])
    R1_read_spline.fit()
    R1_read_positions = [R1_read_spline.value(t) for t in timestamps]
    R1_read_speeds = [R1_read_spline.value(t, der=1) for t in timestamps]
    R1_read_accelerations = [R1_read_spline.value(t, der=2) for t in timestamps]

    R2_read_spline = spline(window_size=WINDOW_SIZE, degree=DEGREE, intersected_values=INTERSECTED_VALUES, x=raw_timestamps, y=data["read_position_2"][:nb_points])
    R2_read_spline.fit()
    R2_read_positions = [R2_read_spline.value(t) for t in timestamps]
    R2_read_speeds = [R2_read_spline.value(t, der=1) for t in timestamps]
    R2_read_accelerations = [R2_read_spline.value(t, der=2) for t in timestamps]

    # Plotting
    plt.subplots(3, 1, sharex=True)
    plt.subplot(3, 1, 1)
    plt.plot(timestamps, R1_read_positions, label="R1")
    plt.plot(timestamps, R2_read_positions, label="R2")
    plt.plot(raw_timestamps, data["read_position_1"][:nb_points], label="R1 raw")
    plt.plot(raw_timestamps, data["read_position_2"][:nb_points], label="R2 raw")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(timestamps, R1_read_speeds, label="R1")
    plt.plot(timestamps, R2_read_speeds, label="R2")
    plt.plot(raw_timestamps, data["read_velocity_1"][:nb_points], label="R1 read speed")
    plt.plot(raw_timestamps, data["read_velocity_2"][:nb_points], label="R2 read speed")
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(timestamps, R1_read_accelerations, label="R1")
    plt.plot(timestamps, R2_read_accelerations, label="R2")
    plt.legend()
    plt.show()


def process_log(src, dst, orientation=np.pi):
    """ Convert the log data to a LogData object """
    with open(src, 'r') as f:
        data = json.load(f)

    log = LogData() 
    raw_timestamps = data["timestamps"]
    log.timestamps = list(np.arange(raw_timestamps[0], raw_timestamps[-1], 1 / SAMPLE_RATE))

    # Initialization
    keys = ["r1", "r2"]
    log.goal_positions = dict.fromkeys(keys)
    log.read_positions = dict.fromkeys(keys)
    log.speeds = dict.fromkeys(keys)
    log.accelerations = dict.fromkeys(keys)
    log.torques = dict.fromkeys(keys)

    # Spline fitting
    print(f"Fitting spline for R1...")
    R1_goal_spline = spline(window_size=WINDOW_SIZE, degree=DEGREE, intersected_values=INTERSECTED_VALUES, x=raw_timestamps, y=data["goal_position_1"])
    R1_goal_spline.fit()
    log.goal_positions["r1"] = [R1_goal_spline.value(t) for t in log.timestamps]

    R1_read_spline = spline(window_size=WINDOW_SIZE, degree=DEGREE, intersected_values=INTERSECTED_VALUES, x=raw_timestamps, y=data["read_position_1"])
    R1_read_spline.fit()
    log.read_positions["r1"] = [R1_read_spline.value(t) for t in log.timestamps]
    log.speeds["r1"] = [R1_read_spline.value(t, der=1) for t in log.timestamps]
    log.accelerations["r1"] = [R1_read_spline.value(t, der=2) for t in log.timestamps]
    print(f"Done !")

    print(f"Fitting spline for R2...")
    R2_goal_spline = spline(window_size=WINDOW_SIZE, degree=DEGREE, intersected_values=INTERSECTED_VALUES, x=raw_timestamps, y=data["goal_position_2"])
    R2_goal_spline.fit()
    log.goal_positions["r2"] = [R2_goal_spline.value(t) for t in log.timestamps]

    R2_read_spline = spline(window_size=WINDOW_SIZE, degree=DEGREE, intersected_values=INTERSECTED_VALUES, x=raw_timestamps, y=data["read_position_2"])
    R2_read_spline.fit()
    log.read_positions["r2"] = [R2_read_spline.value(t) for t in log.timestamps]
    log.speeds["r2"] = [R2_read_spline.value(t, der=1) for t in log.timestamps]
    log.accelerations["r2"] = [R2_read_spline.value(t, der=2) for t in log.timestamps]
    print(f"Done !")

    # Load the robot
    print(f"Torque computation...")
    robot = placo.RobotWrapper('models/2R_arm/', placo.Flags.ignore_collisions)
    robot.set_joint("R1", 1e-5)
    robot.set_joint("R2", 1e-5)
    robot.update_kinematics()
    solver = robot.make_solver()

    # Placing the base in correct orientation
    T_world_base = robot.get_T_world_frame("base")
    T_world_base = T_world_base @ tf.tf.rotation_matrix(orientation, np.array([0., 1., 0.]))
    solver.add_frame_task("base", T_world_base)
    for i in range(10):
        solver.solve(True)
        robot.update_kinematics()

    # Torque estimation
    for i in range(len(log.timestamps)):
        robot.set_joint("R1", log.read_positions["r1"][i])
        robot.set_joint("R2", log.read_positions["r2"][i])
        solver.solve(True)
        robot.update_kinematics()

        qdd = np.array([log.accelerations["r1"][i], log.accelerations["r2"][i]])
        torques_dict = robot.torques_from_acceleration_with_fixed_frame_dict(qdd, "base")

        if log.torques["r1"] is None:
            log.torques["r1"] = [torques_dict["R1"]]
            log.torques["r2"] = [torques_dict["R2"]]
        else:
            log.torques["r1"].append(torques_dict["R1"])
            log.torques["r2"].append(torques_dict["R2"])    
    print(f"Done !")
    
    # Saving log in processed_logs repository
    log.save(dst)


if __name__ == '__main__':

    # Preview spline fitting
    # windows = [30, 33, 36, 39, 42]
    # for windows_size in windows:
    #     WINDOW_SIZE = windows_size
    #     INTERSECTED_VALUES = windows_size - 2
    #     print(f"Window size: {WINDOW_SIZE}, Intersected values: {INTERSECTED_VALUES}")
    #     preview_processing(sys.argv[1])

    excluded_content = []

    for log in os.listdir(sys.argv[1]):
        exlude_log = False
        for content in excluded_content:
            if content in log:
                exlude_log = True
                break
        if exlude_log:
            continue
        
        dst = os.path.join(sys.argv[2], "processed_" + log)
        if os.path.isfile(dst):
            continue

        print(f"Processing {log}...")
        process_log(os.path.join(sys.argv[1], log), dst)
        print(f"Done !")
