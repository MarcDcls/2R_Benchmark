import placo
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
import time

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
z_offset = 0.3
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


def circle_trajectory(t, period, diameter=0.07):
    """Return a 2D circle trajectory."""
    x = diameter/2 * np.sin(2*np.pi * t / period)
    z = diameter/2 * (np.cos(2*np.pi * t / period + np.pi) + 1) + z_offset
    return np.array([x, z])


def line_trajectory(t, period, length=0.15):
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


def smoothstep(t):
    """Smoothing function. Maximum speed in [0, 1] is reached at t=0.5 and is 1.5."""
    return t * t * (3 - 2 * t)


def smooth_trajectory(src, dst, v_max, framerate=0.01):
    """Return a smooth trajectory (list) between src and dst with a maximum speed of v_max."""
    t = 0
    trajectory = []
    c_y = abs(dst - src)
    c_x = v_max / (1.5 * c_y)
    while t < 1 / c_x :
        t += framerate
        scaled_t = t * c_x
        trajectory.append(c_y * smoothstep(scaled_t) * np.sign(dst - src) + src)
    return trajectory


def random_2R_trajectory(duration, max_velocity, framerate=0.01):
    """Return an array containing the motors positions for a random trajectory."""
    l1 = 0.18
    l2 = 0.17
    alpha_trajectory = [0]
    beta_trajectory = [0]

    while len(alpha_trajectory) < duration / framerate:
        alpha = np.random.uniform(-np.pi/2, np.pi/2)
        beta = np.random.uniform(-np.pi/2, np.pi/2)
        while l1 * np.cos(alpha) + l2*np.cos(alpha + beta) < 0:
            beta = np.random.uniform(-np.pi/2, np.pi/2)
        
        v_max = max_velocity * np.random.uniform(0.5, 1)
        delta_alpha = alpha - alpha_trajectory[-1]
        delta_beta = beta - beta_trajectory[-1]
        ratio = abs(delta_alpha / delta_beta)

        if ratio > 1:
            alpha_trajectory += smooth_trajectory(alpha_trajectory[-1], alpha, v_max, framerate)
            beta_trajectory += smooth_trajectory(beta_trajectory[-1], beta, v_max / ratio, framerate)
        else:
            alpha_trajectory += smooth_trajectory(alpha_trajectory[-1], alpha, v_max * ratio, framerate)
            beta_trajectory += smooth_trajectory(beta_trajectory[-1], beta, v_max, framerate)

    return np.array([alpha_trajectory, beta_trajectory]).T


# Testing the trajectories
if __name__ == '__main__':
    # positions = []
    # timeline = np.arange(0, 3, 0.01)
    # for t in timeline:
    #     r1, r2 = get_motors_position(t, traj_type="circle")
    #     pos = robot.get_T_world_frame("end")
    #     positions.append((pos[0, 3], pos[2, 3]))

    # positions = np.array(positions)
    # plt.plot(positions.T[0], positions.T[1])
    # plt.show()



    # src = 90 # deg
    # dst = 30 # deg
    # v_max = 10 # deg/s
    # framerate = 0.01 # s

    # trajectory = smooth_trajectory(src, dst, v_max, framerate)
    # timestamps = np.linspace(0, len(trajectory) * framerate, len(trajectory))
    # plt.plot(timestamps, trajectory)
    # plt.show()


    framerate = 0.01 # s
    trajectory = random_2R_trajectory(20, 5, framerate=framerate)

    # timestamps = np.linspace(0, len(trajectory) * framerate, len(trajectory))
    # plt.plot(timestamps, trajectory)
    # plt.show()

    # speed_alpha = [(trajectory[t+1][0] - trajectory[t][0]) / framerate for t in range(len(trajectory) - 1)]
    # speed_beta = [(trajectory[t+1][1] - trajectory[t][1]) / framerate for t in range(len(trajectory) - 1)]
    # plt.plot(speed_alpha, label="alpha speed")
    # plt.plot(speed_beta, label="beta speed")
    # plt.legend()
    # plt.show()

    from placo_utils.visualization import robot_viz
    viz = robot_viz(robot)
    time.sleep(3)

    for p in trajectory:
        robot.set_joint("R1", p[0])
        robot.set_joint("R2", p[1])
        robot.update_kinematics()
        viz.display(robot.state.q)
        time.sleep(framerate)
