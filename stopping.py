import py_trees
import numpy as np

class Stopping(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Stopping, self).__init__(name)
        self.robot=blackboard.read('robot')
        self.blackboard=blackboard
        
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        
        self.rightMotor = self.robot.getDevice('wheel_right_joint')
        self.leftMotor = self.robot.getDevice('wheel_left_joint')
        self.rightMotor.setPosition(float('Inf'))
        self.leftMotor.setPosition(float('Inf'))
        
        
        
        self.logger.debug("   %s  [Stopping::setup()]" % self.name)
        
    def initialise(self):
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)
        print("Stopping robot")
        return py_trees.common.Status.RUNNING

    def update(self):
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Called when the behavior stops executing."""
        self.logger.debug("%s.terminate(%s)" % (self.name, new_status))