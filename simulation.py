import pybullet as p
import pybullet_data
import time
from camera import setup_and_capture_image
from object_detection import image_object_detection
import cv2

class RobotSimulator:
    def __init__(self):
        self.robot_id, self.cube_id, self.sphere_id = None, None, None
        self.state = "initializing"
        self.end_effector_link_index = 6  # Adjust based on your robot arm
        self.init_simulation()
        self.robot_id, self.cube_id, self.sphere_id = self.load_robot_and_objects()
        self.target_object_id = self.cube_id  # Example: targeting the cube
        self.step_counter = 0
        self.current_joint_positions = None

    def init_simulation(self):
        p.connect(p.GUI)
        p.setGravity(0, 0, -10)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        p.loadURDF("table/table.urdf", basePosition=[0, 0, 0])

    def load_robot_and_objects(self):
        startPos = [-0.4, -0.3, 0.5]
        robot_id = p.loadURDF("kuka_iiwa/model.urdf", startPos)

        # Cube
        cube_start_pos = [0.2, 0, 0.7]
        cube_id = p.loadURDF("cube_small.urdf", cube_start_pos)

        # Sphere
        sphere_start_pos = [-0.2, 0, 0.7]
        sphere_id = p.loadURDF("sphere_small.urdf", sphere_start_pos)

        return robot_id, cube_id, sphere_id

    def move_to_object(self):
        if self.state == "ready_to_move":
            target_pos = p.getBasePositionAndOrientation(self.target_object_id)[0]
            joint_positions = p.calculateInverseKinematics(self.robot_id, self.end_effector_link_index, target_pos)

            # Apply the joint positions gradually for a smoother movement
            if self.current_joint_positions is None:
                self.current_joint_positions = joint_positions

            for i, joint_position in enumerate(joint_positions):
                p.setJointMotorControl2(self.robot_id, i, p.POSITION_CONTROL, targetPosition=joint_position)

            self.state = "moving"

    def check_if_reached(self):
        # This function checks if the robot has reached the target position
        if self.state == "moving":
            # Compare current joint positions to target positions
            current_positions = [p.getJointState(self.robot_id, i)[0] for i in range(p.getNumJoints(self.robot_id))]
            if all(abs(c - t) < 0.01 for c, t in zip(current_positions, self.current_joint_positions)):
                self.state = "ready_to_grab"

    def grab_object(self):
        if self.state == "ready_to_grab":
            # Simulate grabbing by creating a fixed constraint
            p.createConstraint(self.robot_id, self.end_effector_link_index, self.target_object_id, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
            self.state = "holding"

    def update(self):
        if self.state == "initializing":
            self.state = "ready_to_move"
        elif self.state in ["ready_to_move", "moving"]:
            self.move_to_object()
            self.check_if_reached()
        elif self.state == "ready_to_grab":
            self.grab_object()

def run_simulation():
    simulator = RobotSimulator()

    p.stepSimulation()
    time.sleep(1./240.)
    camera_image = setup_and_capture_image(
                            camera_eye_position=[0, 2, 1.6],
                            camera_target_position=[0, 0, 0],
                            camera_up_vector=[0, 0, 1]
                        )

    image = camera_image[:, :, :3]
    print(image.shape)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    image_object_detection(image)
    # try:
    #     while True:
    #         simulator.update()
    #         camera_image = setup_and_capture_image( p
    #                         camera_eye_position=[0, 0, 2],
    #                         camera_target_position=[0, 0, 0],
    #                         camera_up_vector=[-1, 0, 0]
    #                     )
    #         image = np.array(rgbImage)
    #         image = image[:, :, :3]
    #         image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    #         image_object_detection(image)
            
    #         p.stepSimulation()
    #         time.sleep(1./240.)
    # except KeyboardInterrupt:
    #     print("Simulation stopped by user.")
    #     p.disconnect()

run_simulation()
