# --------------------------
# Behavior: Rotate
# Purpose: Adjust robot orientation to center target using differential drive
# --------------------------
import py_trees
import math

class Rotate(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, blackboard, target):
        super().__init__(name)
        self.name = name
        self.robot = blackboard.read('robot')  # Webots robot instance
        self.blackboard = blackboard  # Shared data storage
        # Get alignment tolerance from configuration
        self.target_threshold = self.blackboard.read('centering_threshold')[target]

    def setup(self):
        # Drive system initialization
        self.timestep = int(self.robot.getBasicTimeStep())
        self.motor_speed = 0.2  
        
        # Wheel motor controllers
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.right_motor = self.robot.getDevice('wheel_right_joint')

    def initialise(self):
        # Motor control reset
        self.left_motor.setVelocity(0)  # Stop previous motion
        self.right_motor.setVelocity(0)
        # Set to velocity control mode (infinite position target)
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        print(f'Starting: {self.name}')  # Behavior activation logging

    def update(self):
        # Get latest position error from perception system
        rel_err = self.blackboard.read('target_object_error')
        if rel_err is None:  # No target detected
            self.turn_right()  # Search pattern: slow clockwise rotation
            return py_trees.common.Status.RUNNING

        y_err = rel_err[1]  # Lateral error component (left/right)
        # Check if within centering tolerance
        if math.fabs(y_err) <= self.target_threshold:
            return py_trees.common.Status.SUCCESS  # Properly aligned
            
        # Determine rotation direction based on error sign
        if y_err < 0:  # Target is to the left
            self.turn_right()  # Rotate clockwise to center
        else:  # Target is to the right
            self.turn_left()  # Rotate counter-clockwise

        return py_trees.common.Status.RUNNING  # Continue adjustment

    def turn_right(self):
        # Differential drive: right wheel backward, left wheel forward
        self.right_motor.setVelocity(-self.motor_speed)
        self.left_motor.setVelocity(self.motor_speed)

    def turn_left(self):
        # Differential drive: left wheel backward, right wheel forward
        self.right_motor.setVelocity(self.motor_speed)
        self.left_motor.setVelocity(-self.motor_speed)

    def terminate(self, new_status: py_trees.common.Status):
        # Safety stop - halt wheels regardless of exit status
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)