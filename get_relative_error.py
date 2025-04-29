# --------------------------
# Behavior: GetRelativeError 
# Purpose: Calculate object position error relative to robot using camera recognition
# --------------------------
import py_trees  # Behavior tree library
import numpy as np  

class GetRelativeError(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, blackboard, target):
        super().__init__(name)
        self.robot = blackboard.read('robot')  # Webots robot instance
        self.blackboard = blackboard  # Shared data storage
        # Get target color signature from blackboard configuration
        self.target_color = self.blackboard.read('recognition_colors')[target]

    def setup(self):
        # Camera hardware initialization
        self.timestep = int(self.robot.getBasicTimeStep())
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep)  # Enable camera sensor
        self.camera.recognitionEnable(self.timestep)  # Enable object recognition
        self.object_id = None  # Track first-seen object ID
        
        # Coordinate transformation matrices (camera to robot frame):
        # T_0_1: Base transformation (z-offset for camera height)
        self.T_0_1 = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0.6],  # 0.6m vertical offset
            [0, 0, 0, 1],
        ])
        
        # T_1_2: Forward offset to robot center
        self.T_1_2 = np.array([
            [1, 0, 0, 0.18],  # 0.18m forward offset
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        
        # T_2_3: Final adjustment offset
        self.T_2_3 = np.array([
            [1, 0, 0, 0.005],  # Small x-axis correction
            [0, 1, 0, 0],
            [0, 0, 1, 0.098],  # Z-axis end effector offset
            [0, 0, 0, 1],
        ])

    def initialise(self):
        print(f'Starting: {self.name}')  # Behavior activation logging
        return super().initialise()

    def match_color(self, color1, color2):
        # Strict RGB color matching for object recognition
        for color in range(3):
            if color1[color] != color2[color]:
                return False
        return True

    def update(self):
        # Process camera recognition data
        all_objects = self.camera.getRecognitionObjects()
        # Filter objects by target color signature
        objects = [x for x in all_objects if self.match_color(x.getColors(), self.target_color)]

        # Handle no-detection case
        if len(objects) == 0:
            self.blackboard.write('target_object_error', None)  # Clear previous data
            return py_trees.common.Status.RUNNING  # Keep trying

        # Object tracking logic - maintain focus on first detected instance
        if self.object_id is None:
            self.object_id = objects[0].getId()  # Lock onto first detection

        # Find our tracked object in current frame
        for obj in objects:
            if obj.getId() == self.object_id:
                target_object = obj

        # Transform object position to robot coordinate frame
        p0 = np.array([*list(target_object.getPosition()), 1])  # Homogeneous coords
        # Apply transformation chain: camera → base → effector
        transform_pos = p0 @ self.T_0_1 @ self.T_1_2 @ self.T_2_3  
        self.blackboard.write('target_object_error', transform_pos)  # Share error data

        return py_trees.common.Status.RUNNING  # Continuous monitoring

    def terminate(self, new_status: py_trees.common.Status):
        pass  # No cleanup needed - camera remains active for other behaviors