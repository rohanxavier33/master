# --------------------------
# Behavior: BackUp
# Purpose: Move robot backward for specified distance
# --------------------------
import py_trees  # Behavior tree library

class BackUp(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, blackboard, dist=10):
        super().__init__(name)
        self.name = name
        self.robot = blackboard.read('robot')  # Get robot instance from shared data
        self.blackboard = blackboard  # Reference to shared state storage
        self.motor_speed = 1.14  # Fixed backward speed
        # Calculate required duration: (distance / speed) * time conversion factor
        self.timestep_needed = dist / self.motor_speed * 12  # Empirical timing constant

    def setup(self):
        # Hardware initialization - runs once before first tick
        self.timestep = int(self.robot.getBasicTimeStep())  # Match Webots simulation step
        self.timestep_count = 0  # Reset movement duration counter
        
        # Get motor controller references
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.right_motor = self.robot.getDevice('wheel_right_joint')

    def initialise(self):
        # Motor control setup - runs on first tick
        self.left_motor.setVelocity(0)  # Stop previous motion
        self.right_motor.setVelocity(0)
        # Set position to infinite for velocity control mode
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        print(f'Starting: {self.name}')  # Behavior activation logging

    def update(self):
        # Main execution - called repeatedly until completion
        if self.timestep_count >= self.timestep_needed:
            return py_trees.common.Status.SUCCESS  # Exit condition met
            
        # Set motors to move backward at constant speed
        self.right_motor.setVelocity(-self.motor_speed)
        self.left_motor.setVelocity(-self.motor_speed)

        self.timestep_count += 1  # Track elapsed time
        
        return py_trees.common.Status.RUNNING  # Continue behavior

    def terminate(self, new_status: py_trees.common.Status):
        # Cleanup - always called after behavior completes
        self.left_motor.setVelocity(0)  # Stop motors
        self.right_motor.setVelocity(0)