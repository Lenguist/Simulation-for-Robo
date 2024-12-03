import pybullet as p
import time
import pybullet_data
import math
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,0)
groundId = p.loadURDF("plane.urdf")
robotStartPos = [0,0,1] 
robotStartOrientation = p.getQuaternionFromEuler([math.radians(90),0,0])  # 90 degrees yaw, bc i think stl file setup like this, but this is a fine workaround
robotId = p.loadURDF("test_robot2.urdf",robotStartPos, robotStartOrientation)
mode = p.POSITION_CONTROL
jointIndex = 0 # first joint is number 0

# camera control stuff
cameraDistance = 0.75  # Smaller values zoom in closer to the robot
cameraYaw = 60        # Rotate the camera around the robot
cameraPitch = -30     # Tilt the camera up/down
cameraTargetPosition = [0, 0, 1]  # Focus the camera on the robot
p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)


for i in range (10000):  
    # for camera control
    robotPos, _ = p.getBasePositionAndOrientation(robotId)
    p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, robotPos)  # Follow the robot

    p.setJointMotorControl2(robotId, jointIndex, controlMode=mode, targetPosition=0.8+0.6*math.sin(i*0.01))
    p.stepSimulation()
    time.sleep(1./2400.)
robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
print(robotPos, robotOrn)
p.disconnect()