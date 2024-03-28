import pybullet as p
import pybullet_data
import numpy as np


def setup_and_capture_image(camera_eye_position, camera_target_position, camera_up_vector, width=640, height=480, fov=60, aspect=1.0, near=0.02, far=5):
    """
    Set up a camera in the PyBullet simulation and capture an RGB image from that viewpoint.
    
    :param camera_eye_position: The position of the camera in the simulation.
    :param camera_target_position: The point the camera is looking at.
    :param camera_up_vector: The 'up' direction for the camera.
    :param width: The width of the captured image.
    :param height: The height of the captured image.
    :param fov: The field of view for the camera.
    :param aspect: The aspect ratio of the camera.
    :param near: The near clipping plane for the camera.
    :param far: The far clipping plane for the camera.
    :return: An RGB image captured from the simulation.
    """
    # Compute view and projection matrices
    view_matrix = p.computeViewMatrix(cameraEyePosition=camera_eye_position, cameraTargetPosition=camera_target_position, cameraUpVector=camera_up_vector)
    projection_matrix = p.computeProjectionMatrixFOV(fov=fov, aspect=aspect, nearVal=near, farVal=far)
    
    # Capture an image from the simulation
    img_arr = p.getCameraImage(width, height, viewMatrix=view_matrix, projectionMatrix=projection_matrix)
    
    # Extract the RGB part of the image
    rgb = img_arr[2]
    rgb_image = np.reshape(rgb, (height, width, 4))
    
    return rgb_image
