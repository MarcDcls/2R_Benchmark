import placo
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt

# Load the robot
robot = placo.RobotWrapper('2R_arm/', placo.Flags.ignore_collisions)
robot.set_joint("R1", 1e-5)
robot.update_kinematics()
solver = robot.make_solver()

# Base is fixed
T_world_base = robot.get_T_world_frame("base")
base_task = solver.add_frame_task("base", T_world_base)
base_task.configure("base", "hard", 1., 1.)

# Creating end task
y_offset = robot.get_T_world_frame("end")[1, 3]
z_offset = 0.2
end_task = solver.add_position_task("end", np.array([0., y_offset, z_offset]))
end_task.configure("end", "soft", 1.0)

# Reaching initial configuration
for i in range(10):
    solver.solve(True)
    robot.update_kinematics()
initial_R1 = robot.get_joint("R1")
initial_R2 = robot.get_joint("R2")

def initial_configuration():
    """Return the initial configuration of the robot."""
    return np.array([initial_R1, initial_R2])

def circle_trajectory(t, period, diameter=0.04):
    """Return a 2D circle trajectory."""
    x = diameter/2 * np.sin(2*np.pi * t / period)
    z = diameter/2 * (np.cos(2*np.pi * t / period + np.pi) + 1) + z_offset
    return np.array([x, z])

def line_trajectory(t, period, length=0.05):
    """Return a 2D line trajectory."""
    x = length/2 * np.sin(2*np.pi * t / period)
    return np.array([x, z_offset])

def get_motors_position(t, period, traj_type):
    """Return the motors position for the given trajectory."""

    if traj_type == "line":
        x, z = line_trajectory(t, period=period)
    elif traj_type == "circle":
        x, z = circle_trajectory(t, period=period)
    else:
        raise ValueError("Unknown trajectory type: %s" % traj_type)
    
    end_task.target_world = np.array([x, y_offset, z])
    solver.solve(True)
    robot.update_kinematics()
    return robot.get_joint("R1"), robot.get_joint("R2")

def get_motors_positions(duration, period=1, traj_type="line", framerate=0.01):
    """Return an array containing the motors positions for the given trajectory."""
    positions = []
    for t in np.arange(0, duration, framerate):
        r1, r2 = get_motors_position(t, traj_type=traj_type, period=period)
        positions.append((r1, r2))
    return np.array(positions)

# Testing the trajectories
if __name__ == '__main__':
    positions = []
    timeline = np.arange(0, 3, 0.01)
    for t in timeline:
        r1, r2 = get_motors_position(t, traj_type="circle")
        pos = robot.get_T_world_frame("end")
        positions.append((pos[0, 3], pos[2, 3]))

    positions = np.array(positions)
    plt.plot(positions.T[0], positions.T[1])
    plt.show()