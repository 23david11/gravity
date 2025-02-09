import pybullet as p
import pybullet_data
import time
import math

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)

planeID = p.loadURDF("plane.urdf")

euler_angles = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler(euler_angles)
start_position = [0, 0, 3]

robotID = p.loadURDF("ball.urdf", start_position, startOrientation)

v = 0
g = 9.8
t = 1./240.
z = start_position[2]
e = 0.8

for i in range (100000):

    if (z >= 0):
        start_position = [0, 0, z]

        p.resetBasePositionAndOrientation(robotID, start_position, startOrientation)

        z = z + v * t - g/2 * t**2
        v = v - g * t
    else:
        z = 0
        v = -v * e

    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()