import pybullet as p
import time
import pybullet_data
import math

# Connect to PyBullet
physicsClient = p.connect(p.GUI)  # Use p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Optionally set search path for PyBullet data
p.setGravity(0, 0, 0)

# Load the ground plane
planeId = p.loadURDF("plane.urdf")

# Define positions and orientations for the two URDFs
robot1StartPos = [0, 0, 1]  # Position for the first robot
robot1StartOrientation = p.getQuaternionFromEuler([math.radians(90), 0, 0])  # Orientation for the first robot

robot2StartPos = [0.5, 0, 1]  # Position for the second robot (adjust as needed)
robot2StartOrientation = p.getQuaternionFromEuler([math.radians(90), 0, 0])  # Orientation for the second robot

# Load the first robot URDF
robot1Id = p.loadURDF("test_robot.urdf", robot1StartPos, robot1StartOrientation)

# Load the second robot URDF
robot2Id = p.loadURDF("test_robot2.urdf", robot2StartPos, robot2StartOrientation)

# Camera control parameters
cameraDistance = 1.0  # Adjust as needed
cameraYaw =80  # Rotate the camera around the scene
cameraPitch = -30  # Tilt the camera up/down
cameraTargetPosition = [0.25, 0, 1]  # Focus the camera between the two robots
p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)

# Simulation loop
for i in range(10000):
    # Update the camera to follow the midpoint between the two robots
    robot1Pos, _ = p.getBasePositionAndOrientation(robot1Id)
    robot2Pos, _ = p.getBasePositionAndOrientation(robot2Id)
    midpoint = [(robot1Pos[0] + robot2Pos[0]) / 2, (robot1Pos[1] + robot2Pos[1]) / 2, (robot1Pos[2] + robot2Pos[2]) / 2]
    p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, midpoint)

    # Step the simulation
    p.stepSimulation()
    time.sleep(1. / 2400.)

# Print final positions and orientations
robot1Pos, robot1Orn = p.getBasePositionAndOrientation(robot1Id)
robot2Pos, robot2Orn = p.getBasePositionAndOrientation(robot2Id)
print("Robot 1 Position and Orientation:", robot1Pos, robot1Orn)
print("Robot 2 Position and Orientation:", robot2Pos, robot2Orn)

# Disconnect from PyBullet
p.disconnect()
