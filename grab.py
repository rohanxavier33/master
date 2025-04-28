import py_trees
import numpy as np

class Grab(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Grab, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        return py_trees.common.Status.RUNNING

       
    
    def initialise(self):
        pass
        
    def update(self):
        
        # Initialize motors and set target positions
        self.timestep = int(self.robot.getBasicTimeStep())

        self.gripper_left_finger_joint = self.robot.getDevice('gripper_left_finger_joint')
        self.gripper_right_finger_joint = self.robot.getDevice('gripper_right_finger_joint')

        self.gripper_left_finger_joint.enableForceFeedback(self.timestep)
        self.gripper_right_finger_joint.enableForceFeedback(self.timestep)

        leftForce = self.gripper_left_finger_joint.getForceFeedback()
        rightForce = self.gripper_right_finger_joint.getForceFeedback()

        if leftForce <= -10 or rightForce <= -10:
            print("Grabbed object")
            return py_trees.common.Status.SUCCESS
        else:
            self.gripper_left_finger_joint.setPosition(0.0)
            self.gripper_right_finger_joint.setPosition(0.0)
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("%s.terminate(%s)" % (self.name, new_status))