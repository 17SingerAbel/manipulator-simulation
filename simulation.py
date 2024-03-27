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
    # Add a cube
    cube_start_pos = [0.2, 0, 0.6]
    cube_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
    cube_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05], rgbaColor=[1, 0, 0, 1])
    cube_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=cube_collision_shape, baseVisualShapeIndex=cube_visual_shape, basePosition=cube_start_pos)

    # Add a sphere
    sphere_start_pos = [-0.2, 0, 0.6]
    sphere_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.05)
    sphere_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 0, 1, 1])
    sphere_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=sphere_collision_shape, baseVisualShapeIndex=sphere_visual_shape, basePosition=sphere_start_pos)


    return robot_id, cube_id, sphere_id

def move_to_object(robot_id, target_pos):
    """Move the robot arm to a predefined position above an object."""
    # This function would use inverse kinematics and other logic to position the arm
    pass  # Implement movement logic here

def grab_and_sort_object(robot_id, object_id, target_pos):
    """Grab an object and move it to a new location based on its type."""
    # This would involve closing the gripper and moving the object
    pass  # Implement grabbing and sorting logic here

        

def run_simulation():
    init_simulation()
    robot_id, cube_id, sphere_id = load_robot_and_objects()
    # robot_id = load_robot_and_objects()

    # Example: move to and sort the cup
    # move_to_object(robot_id, [0.7, 0, 0.76])
    # grab_and_sort_object(robot_id, cup_id, [0.5, 0.5, 0.76])  # Example target position

    # Add logic for other objects as needed
    # Camera 1: Top-down view
    top_camera = setup_and_capture_image(
        camera_eye_position=[0, 0, 2],
        camera_target_position=[0, 0, 0],
        camera_up_vector=[-1, 0, 0]
    )

    # # Camera 2: Side view
    # side_camera = setup_and_capture_image(
    #     camera_eye_position=[1, 1, 1],
    #     camera_target_position=[0, 0, 0.5],
    #     camera_up_vector=[0, 0, 1]
    # )

    try:
        while True:
            p.stepSimulation()
            time.sleep(1./240.)
    except KeyboardInterrupt:
        p.disconnect()

run_simulation()
