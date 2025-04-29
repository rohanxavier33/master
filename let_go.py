import py_trees as pt
import numpy as np

class LetGo(pt.behaviour.Behaviour):
    def __init__(self, name: str, blackboard):
        super().__init__(name)
        self.robot = blackboard.read('robot')
        self.desired_pose = {
            'torso_lift_joint': 0.20,
            'arm_1_joint': 1.57,
            'arm_2_joint': 0.314,
            'arm_3_joint': 0,
            'arm_4_joint': 0.35,
            'arm_5_joint': 0,
            'arm_6_joint': 0,
            'arm_7_joint': 1.57,
            'head_1_joint': 0,
            'head_2_joint': 0,
        }
        self.gripper_pose = {
            'gripper_left_finger_joint': 0.045,
            'gripper_right_finger_joint': 0.045
        }
        self.arm_positioned = False

    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Initialize arm joint motors and encoders
        self.motors = {}
        self.encoders = {}
        for joint_name in self.desired_pose:
            self.motors[joint_name] = self.robot.getDevice(joint_name)
            self.encoders[joint_name] = self.robot.getDevice(joint_name + "_sensor")
            self.encoders[joint_name].enable(self.timestep)
            
        # Initialize gripper joints and encoders
        self.gripper_motors = {}
        self.gripper_encoders = {}
        for joint_name in self.gripper_pose:
            self.gripper_motors[joint_name] = self.robot.getDevice(joint_name)
            encoder_name = joint_name.replace('finger_joint', 'sensor_finger_joint')
            self.gripper_encoders[joint_name] = self.robot.getDevice(encoder_name)
            self.gripper_encoders[joint_name].enable(self.timestep)

    def initialise(self):
        # Set initial target positions for arm joints
        for joint_name, position in self.desired_pose.items():
            self.motors[joint_name].setPosition(position)

    def update(self):
        if not self.arm_positioned:
            # Check if arm has reached desired position
            sum_sq_error = 0.0
            for joint_name, target in self.desired_pose.items():
                current = self.encoders[joint_name].getValue()
                error = current - target
                sum_sq_error += error ** 2

            if sum_sq_error < 0.001:  # Arm is in position
                self.arm_positioned = True
                # Start opening gripper
                for joint_name, position in self.gripper_pose.items():
                    self.gripper_motors[joint_name].setPosition(position)
            return pt.common.Status.RUNNING
        
        else:
            # Check if gripper has fully opened
            gripper_sum_sq_error = 0.0
            for joint_name, target in self.gripper_pose.items():
                current = self.gripper_encoders[joint_name].getValue()
                error = current - target
                gripper_sum_sq_error += error ** 2
            
            if gripper_sum_sq_error < 0.0000001:  # Gripper is fully open
                return pt.common.Status.SUCCESS
            return pt.common.Status.RUNNING

    def terminate(self, new_status):
        pass