import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import numpy as np
import time
from camera import setup_and_capture_image

def init_simulation():
    p.connect(p.GUI)
    p.setGravity(0, 0, -10)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = p.loadURDF("plane.urdf")
    table_id = p.loadURDF("table/table.urdf", basePosition=[0, 0, 0])

    return plane_id, table_id

def load_robot_and_objects():
    startPos = [-0.4, -0.3, 0.5]  # Adjust height based on table and robot model
    robot_id = p.loadURDF("kuka_iiwa/model.urdf", startPos)

    # Assuming generic shapes for cup and plate; replace with custom URDFs if available
    # Cube
    # Add a cube
    cube_start_pos = [0.2, 0, 0.7]
    cube_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
    cube_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05], rgbaColor=[1, 0, 0, 1])
    cube_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=cube_collision_shape, baseVisualShapeIndex=cube_visual_shape, basePosition=cube_start_pos)

    # Add a sphere
    sphere_start_pos = [-0.2, 0, 0.7]
    sphere_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.05)
    sphere_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 0, 1, 1])
    sphere_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=sphere_collision_shape, baseVisualShapeIndex=sphere_visual_shape, basePosition=sphere_start_pos)


    return robot_id, cube_id, sphere_id

def move_to_object_and_grab(robot_id, object_id, end_effector_link_index=6):
    target_pos = p.getBasePositionAndOrientation(object_id)[0]
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])
    joint_positions = p.calculateInverseKinematics(robot_id, end_effector_link_index, target_pos, target_orientation)

    # Gradually apply the IK solution to animate the arm movement
    for coef in np.linspace(0, 1, num=1000):
        current_joint_positions = [coef * jp for jp in joint_positions]
        for joint_index, joint_pos in enumerate(current_joint_positions):
            p.setJointMotorControl2(robot_id, joint_index, p.POSITION_CONTROL, targetPosition=joint_pos)
        p.stepSimulation()
        time.sleep(1./240.)

def run_simulation():
    init_simulation()
    robot_id, cube_id, sphere_id = load_robot_and_objects()
    p.stepSimulation()
    time.sleep(1./240.)

    try:
        while True:
            top_camera = setup_and_capture_image(
                camera_eye_position=[0, 0, 2],
                camera_target_position=[0, 0, 0],
                camera_up_vector=[-1, 0, 0]
            )
            p.stepSimulation()
            time.sleep(1./240.)
    except KeyboardInterrupt:
        p.disconnect()

run_simulation()
