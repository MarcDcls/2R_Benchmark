import placo
import numpy as np


# Load the robot
robot = placo.RobotWrapper('1R_arm/', placo.Flags.ignore_collisions)
robot.set_joint("R1", 1e-5)
robot.update_kinematics()
solver = robot.make_solver()

robot.static_gravity_compensation_torques

J_world_base = robot.frame_jacobian("base", "world")
# print("J_base :", J_world_base)

M = robot.mass_matrix()
h = robot.non_linear_effects()
g = robot.generalized_gravity()

qdd = np.array([1.])

tau_m = M[6:, 6:] @ qdd + g[6:] - J_world_base.T @ np.array([0., 0., 1.])
