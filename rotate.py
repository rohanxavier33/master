import py_trees as pt
import numpy as np
import math

class Rotate(pt.behaviour.Behaviour):
    def __init__(self, name: str, blackboard, target):
        super().__init__(name)
        self.name = name
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        self.target_threshold = self.blackboard.read('centering_threshold')[target]

    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        self.rotation_speed = 0.8

        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.right_motor = self.robot.getDevice('wheel_right_joint')

    def initialise(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        print(f'Starting: {self.name}')

    def update(self):
        # this value should be written by `GetRelativeErrorToObject` behavior
        rel_err = self.blackboard.read('target_object_error')
        if rel_err is None:
            # turn right by default, since robot must find target object
            self.turn_right()
            return pt.common.Status.RUNNING

        y_err = rel_err[1]
        if math.fabs(y_err) <= self.target_threshold:
            return pt.common.Status.SUCCESS

        if y_err < 0:
            self.turn_right()
        else:
            self.turn_left()

        return pt.common.Status.RUNNING

    def turn_right(self):
        self.right_motor.setVelocity(-self.rotation_speed)
        self.left_motor.setVelocity(self.rotation_speed)

    def turn_left(self):
        self.right_motor.setVelocity(self.rotation_speed)
        self.left_motor.setVelocity(-self.rotation_speed)

    def terminate(self, new_status: pt.common.Status):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

