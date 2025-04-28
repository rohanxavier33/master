import py_trees
import numpy as np

# Define a threshold for considering the joints to be in position
JOINT_ERROR_THRESHOLD = 0.005 # You might need to tune this value

class LetGo(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(LetGo, self).__init__(name)
        # It's generally better practice to read from the blackboard
        # closer to when you need it (e.g., in setup or initialise)
        # unless you are certain it's populated when __init__ runs.
        # However, sticking to the original structure for minimal changes.
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        self.motors = {}
        self.encoders = {}
        self.gripper_motors = {} # Separate dictionary for gripper motors
        self.target_positions = {} # Will be populated in initialise

    def setup(self):
        self.logger.debug("%s.setup()" % self.name)
        try:
            self.timestep = int(self.robot.getBasicTimeStep())

            # --- Moved Initializations ---
            # Define target joints (excluding gripper for the main pose)
            # Note: Gripper targets are handled separately later
            self.robot_joint_names = [
                'torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
                'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint',
                'head_1_joint', 'head_2_joint'
            ]
            # Gripper joint names
            self.gripper_joint_names = [
                'gripper_left_finger_joint', 'gripper_right_finger_joint'
            ]

            # Initialize motors for main joints
            for name in self.robot_joint_names:
                motor = self.robot.getDevice(name)
                if motor is None:
                    self.logger.error(f"Motor '{name}' not found.")
                    return py_trees.common.Status.FAILURE
                self.motors[name] = motor

            # Initialize encoders for main joints
            for name in self.robot_joint_names:
                sensor_name = name + "_sensor"
                encoder = self.robot.getDevice(sensor_name)
                if encoder is None:
                     # Fallback for potential naming conventions or simple joints without sensors
                     # This part might need adjustment based on your specific robot model in Webots
                     self.logger.warning(f"Encoder '{sensor_name}' not found for main joint '{name}'. This might cause issues if position checking relies on it.")
                     # Add a placeholder or handle appropriately if needed. For now, we'll skip enabling.
                     continue # Skip enabling if not found
                self.encoders[name] = encoder
                self.encoders[name].enable(self.timestep)

            # Initialize gripper motors
            for name in self.gripper_joint_names:
                 motor = self.robot.getDevice(name)
                 if motor is None:
                    self.logger.error(f"Gripper motor '{name}' not found.")
                    return py_trees.common.Status.FAILURE
                 self.gripper_motors[name] = motor
            # Note: We don't necessarily need gripper encoders if we just command open

            # Optional: Initialize GPS and Compass (if needed elsewhere, otherwise can be removed)
            self.gps = self.robot.getDevice('gps')
            if self.gps: self.gps.enable(self.timestep)
            self.compass = self.robot.getDevice('compass')
            if self.compass: self.compass.enable(self.timestep)
            # --- End Moved Initializations ---

            return py_trees.common.Status.SUCCESS # Standard setup return

        except Exception as e:
            self.logger.error(f"Error during setup: {e}")
            return py_trees.common.Status.FAILURE


    def initialise(self):
        self.logger.debug("%s.initialise()" % self.name)
        # Set target positions for the main joints
        self.target_positions = {
            'torso_lift_joint': 0.20,
            'arm_1_joint': 1.57,
            'arm_2_joint': 0.314,
            'arm_3_joint': 0,
            'arm_4_joint': 0.35,
            'arm_5_joint': 0,
            'arm_6_joint': 0,
            'arm_7_joint': 1.57,
            'head_1_joint': 0,
            'head_2_joint': 0
        }
        # No need to re-initialize motors/encoders here

    def update(self):
        # 1. Command main joints to target positions
        for name, target_pos in self.target_positions.items():
             if name in self.motors:
                 self.motors[name].setPosition(target_pos)
             else:
                 # This should not happen if setup was successful
                 self.logger.warning(f"Motor '{name}' not found during update.")
                 # Depending on criticality, you might return FAILURE here

        # 2. Check if main joints have reached the target pose
        sum_sq_error = 0.0
        joint_count = 0
        for jname, target in self.target_positions.items():
            if jname in self.encoders: # Only calculate error if encoder exists
                current = self.encoders[jname].getValue()
                error = current - target
                sum_sq_error += error ** 2
                joint_count += 1
            else:
                 # If no encoder, we can't check this joint's position.
                 # Decide how to handle this: assume it's okay, or fail?
                 # For now, we just skip it in the error calculation.
                 pass

        # Avoid division by zero if no encoders were found/used
        average_sq_error = (sum_sq_error / joint_count) if joint_count > 0 else 0.0

        # 3. Determine status based on error
        if average_sq_error >= JOINT_ERROR_THRESHOLD:
            # Not yet in position, keep running
            # self.logger.debug(f"Moving to pose, error: {average_sq_error:.4f}") # Optional debug log
            return py_trees.common.Status.RUNNING
        else:
            # Pose reached! Now command the gripper to open.
            self.logger.info("Main pose reached. Opening gripper.")
            gripper_open_position = 0.045 # Target position for open gripper fingers
            if 'gripper_left_finger_joint' in self.gripper_motors:
                 self.gripper_motors['gripper_left_finger_joint'].setPosition(gripper_open_position)
            if 'gripper_right_finger_joint' in self.gripper_motors:
                 self.gripper_motors['gripper_right_finger_joint'].setPosition(gripper_open_position)

            # Since the goal is achieved (pose reached + gripper command sent), return SUCCESS
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("%s.terminate(%s)" % (self.name, new_status))
        # Optional: You could disable sensors here if needed,
        # but often it's fine to leave them enabled.
        # for encoder in self.encoders.values():
        #     encoder.disable()
        # if self.gps: self.gps.disable()
        # if self.compass: self.compass.disable()