import py_trees
import numpy as np

class Safe(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Safe, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        print("Safe behaviour initialized")
       
    def setup(self):
        print("Safe.setup() called")
        self.timestep = int(self.robot.getBasicTimeStep())
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        print("Safe behaviour setup complete")
        return py_trees.common.Status.RUNNING
    
    def initialise(self):
       print("Safe.initialise() called")

    def update(self):
         # Define target positions for each joint
        self.robot_joints = {
            'torso_lift_joint': 0.35,
            'arm_1_joint': 0.71,
            'arm_2_joint': 1.02,
            'arm_3_joint': -2.815,
            'arm_4_joint': 1.011,
            'arm_5_joint': 0,
            'arm_6_joint': 0,
            'arm_7_joint': 0,
            'gripper_left_finger_joint': 0,
            'gripper_right_finger_joint': 0,
            'head_1_joint': 0,
            'head_2_joint': 0
        }
        # Initialize motors and set target positions
        self.motors = {}
        for name in self.robot_joints:
            self.motors[name] = self.robot.getDevice(name)
            self.motors[name].setPosition(self.robot_joints[name])
        
        # Initialize encoders for each joint
        self.encoders = {}
        for jname in self.robot_joints:
            # Handle special case for finger joints
            if jname in ['gripper_left_finger_joint', 'gripper_right_finger_joint']:
                sensor_name = jname.replace('finger_joint', 'sensor_finger_joint')
            else:
                sensor_name = jname + "_sensor"
            self.encoders[jname] = self.robot.getDevice(sensor_name)
            self.encoders[jname].enable(self.timestep)
        
        # Store target positions for comparison
        self.target_positions = self.robot_joints.copy()
        sum_sq_error = 0.0
        for jname, target in self.target_positions.items():
            current = self.encoders[jname].getValue()
            error = current - target
            sum_sq_error += error ** 2
           
       
        if sum_sq_error < 0.001:
            print("Safe position reached, returning SUCCESS")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("%s.terminate(%s)" % (self.name, new_status))