import placo
import numpy as np
import time
import placo_utils.tf as tf


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


def torques(qdd_a):
    h = robot.non_linear_effects()
    M = robot.mass_matrix()

    # Compute f the forces fixing the frame
    J = robot.frame_jacobian("base", "world")
    f = np.linalg.inv(J.T[:6, :6]) @ h[:6]

    h_a = h[6:]
    M_a = M[6:, 6:]
    
    return M_a @ qdd_a + h_a - (J.T @ f)[6:]


t0 = time.time()
t = time.time() - t0
while t < 2:
    robot.set_joint("R1", np.pi/2 * np.sin(t * np.pi/2))
    solver.solve(True)
    robot.update_kinematics()

    print(torques(np.array([1.])))
    print(robot.torques_from_acceleration_with_fixed_frame(np.array([1.]), "base"))

    time.sleep(0.01)
    t = time.time() - t0