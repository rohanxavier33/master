# --------------------------
# Behavior: Grab
# Purpose: Control gripper mechanism to grasp objects using force feedback
# --------------------------
import py_trees
import numpy as np

class Grab(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Grab, self).__init__(name)
        self.robot = blackboard.read('robot')  # Webots robot instance
        self.blackboard = blackboard  # Shared data storage
        
    def setup(self):
        # Sensor initialization for positioning verification
        self.timestep = int(self.robot.getBasicTimeStep())
        self.gps = self.robot.getDevice('gps')  # Ground position sensor
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')  # Orientation sensor
        self.compass.enable(self.timestep)
        return py_trees.common.Status.RUNNING

    def initialise(self):
        # No special initialization needed - empty implementation
        pass
        
    def update(self):
        # Main gripping control logic
        # Hardware interface setup
        self.timestep = int(self.robot.getBasicTimeStep())

        # Get gripper joint controllers
        self.gripper_left_finger_joint = self.robot.getDevice('gripper_left_finger_joint')
        self.gripper_right_finger_joint = self.robot.getDevice('gripper_right_finger_joint')

        # Enable force feedback sensing
        self.gripper_left_finger_joint.enableForceFeedback(self.timestep)
        self.gripper_right_finger_joint.enableForceFeedback(self.timestep)

        # Read current grip forces (negative values indicate contact)
        leftForce = self.gripper_left_finger_joint.getForceFeedback()
        rightForce = self.gripper_right_finger_joint.getForceFeedback()

        # Check if object is detected through force thresholds
        if leftForce <= -10 or rightForce <= -10:
            print("Grabbed object")  # Success confirmation
            return py_trees.common.Status.SUCCESS  # Gripping complete
        else:
            # Close gripper by moving fingers to closed position (0.0)
            self.gripper_left_finger_joint.setPosition(0.0)
            self.gripper_right_finger_joint.setPosition(0.0)
            return py_trees.common.Status.RUNNING  # Continue gripping attempt

    def terminate(self, new_status):
        # Standard behavior cleanup with debug logging
        self.logger.debug("%s.terminate(%s)" % (self.name, new_status))