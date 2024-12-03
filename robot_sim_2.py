import pybullet as p
import time
import pybullet_data
import math

# Connect to PyBullet
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, 0)
groundId = p.loadURDF("plane.urdf")

# Load the robot URDF
robotStartPos = [0, 0, 1]
robotStartOrientation = p.getQuaternionFromEuler([math.radians(90), 0, 0])  # Adjust orientation
robotId = p.loadURDF("test_robot2.urdf", robotStartPos, robotStartOrientation)

# Camera control parameters
cameraDistance = 0.75  # Smaller values zoom in closer to the robot
cameraYaw = 60         # Rotate the camera around the robot
cameraPitch = -30      # Tilt the camera up/down
cameraTargetPosition = [0, 0, 1]  # Focus the camera on the robot
p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)

# Control mode
mode = p.POSITION_CONTROL

# Get the number of joints in the robot
numJoints = p.getNumJoints(robotId)
print(f"Number of joints: {numJoints}")

# Main simulation loop
for i in range(10000):
    # Update the camera to follow the robot
    robotPos, _ = p.getBasePositionAndOrientation(robotId)
    p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, robotPos)

    # Move all joints
    for jointIndex in range(numJoints):
        targetPosition = 0.8 + 1.6 * math.sin(i * 0.01 + jointIndex)  # Offset the phase for each joint
        p.setJointMotorControl2(robotId, jointIndex, controlMode=mode, targetPosition=targetPosition)

    # Step the simulation
    p.stepSimulation()
    time.sleep(1. / 2400.)

# Print final position and orientation
robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
print(robotPos, robotOrn)

# Disconnect from PyBullet
p.disconnect()