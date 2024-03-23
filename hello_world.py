import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setGravity(0, 0, -10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # to load default models

planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("kuka_iiwa/model.urdf", startPos, startOrientation)


# scene setup

# Cube
cubeStartPos = [0.5, 0, 0.5]
cubeUid = p.loadURDF("cube_small.urdf", cubeStartPos)

# Sphere
sphereStartPos = [0, 0.5, 0.5]
sphereRadius = 0.1
sphereMass = 1
sphereVisualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=sphereRadius, rgbaColor=[1, 0, 0, 1])
sphereCollisionShapeId = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=sphereRadius)
sphereUid = p.createMultiBody(baseMass=sphereMass, baseCollisionShapeIndex=sphereCollisionShapeId, baseVisualShapeIndex=sphereVisualShapeId, basePosition=sphereStartPos)

# Cylinder
cylinderStartPos = [-0.5, 0, 0.5]
cylinderRadius = 0.1
cylinderHeight = 0.2
cylinderMass = 1
cylinderVisualShapeId = p.createVisualShape(shapeType=p.GEOM_CYLINDER, radius=cylinderRadius, length=cylinderHeight, rgbaColor=[0, 1, 0, 1])
cylinderCollisionShapeId = p.createCollisionShape(shapeType=p.GEOM_CYLINDER, radius=cylinderRadius, height=cylinderHeight)
cylinderUid = p.createMultiBody(baseMass=cylinderMass, baseCollisionShapeIndex=cylinderCollisionShapeId, baseVisualShapeIndex=cylinderVisualShapeId, basePosition=cylinderStartPos)


# Example of moving the robot arm - this would be replaced by your grab logic
# This is a very simplistic approach, just for demonstration
jointPositions = [0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.7]
for jointIndex in range(p.getNumJoints(robotId)):
    p.setJointMotorControl2(robotId, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex])

# Simulate for a short period
try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
except KeyboardInterrupt:
    print("Simulation stopped by user.")
    p.disconnect()