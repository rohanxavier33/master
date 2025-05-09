import py_trees
import math

class StraightForward(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, blackboard, threshold=0.85):
        super().__init__(name)
        self.name = name
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        self.threshold = threshold

    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        self.motor_speed = 1.0

        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.right_motor = self.robot.getDevice('wheel_right_joint')

    def initialise(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        print(f'Starting: {self.name}')

    def update(self):
        rel_err = self.blackboard.read('target_object_error')
        if rel_err is None:
            self.off_val = 0
            return py_trees.common.Status.RUNNING
        else:
            self.off_val = 0.03

        x_err = rel_err[0]
        y_err = rel_err[1] + self.off_val
        if math.fabs(x_err) <= self.threshold:
            return py_trees.common.Status.SUCCESS

        self.left_motor.setVelocity(self.motor_speed - y_err)
        self.right_motor.setVelocity(self.motor_speed + y_err)

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

