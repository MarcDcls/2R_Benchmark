import pybullet as p
from placo_utils.visualization import robot_viz, frame_viz, point_viz, robot_frame_viz
import time
import numpy as np
from trajectory import get_motors_position, robot
import argparse

# Parsing arguments with long and short options
parser = argparse.ArgumentParser()
parser.add_argument("--framerate", "-f", help="Framerate", default=0.01, type=float)
parser.add_argument("--trajectory", "-t", help="Trajectory type (circle or line)", default="circle")
parser.add_argument("--pybullet", "-p", help="Use pybullet viewer", default=False, action="store_true")
args = parser.parse_args()

if args.pybullet:
    physicsClient = p.connect(p.GUI)
    startPos = [0, 0, 1]
    startOrientation = p.getQuaternionFromEuler([np.pi, 0, 0])
    armId = p.loadURDF("2R_arm/robot.urdf", startPos, startOrientation, useFixedBase=True)
    # p.setGravity(0,0,-10)
else:
    viz = robot_viz(robot)

for i in range (10000):
    # Set the motors position
    t = i * args.framerate
    q = get_motors_position(t, traj_type=args.trajectory, period=1)

    # Vizualization
    if args.pybullet:
        p.stepSimulation()
        p.setJointMotorControl2(armId, 0, p.POSITION_CONTROL, targetPosition=q[0])
        p.setJointMotorControl2(armId, 1, p.POSITION_CONTROL, targetPosition=q[1])
    else:
        viz.display(robot.state.q)
        robot_frame_viz(robot, "end")

    time.sleep(args.framerate)

if args.pybullet:
    p.disconnect()
