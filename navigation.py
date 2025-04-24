"""navigation controller."""

import py_trees
import numpy as np

class Navigation(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Navigation, self).__init__(name)
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
        
       
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)
        self.logger.debug("   %s  [Navigation::setup()]" % self.name)
        
    def initialise(self):
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)
        
        self.index=0
        
        self.WP=self.blackboard.read('waypoints')
        
    def update(self):
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        theta=np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])
        
        rho = np.sqrt((xw-self.WP[self.index][0])**2 + (yw-self.WP[self.index][1])**2)
        alpha = np.arctan2(self.WP[self.index][1] -yw, self.WP[self.index][0]-xw) - theta
        
        if alpha > np.pi:
            alpha -= 2*np.pi
        if alpha < -np.pi:
            alpha += 2*np.pi
            
        
        p1, p2 = 5, 5  # Controller gains
        leftSpeed = - alpha*p1 + rho*p2
        rightSpeed = alpha*p1 + rho*p2
        
        # Clamp motor speeds to maximum allowed
        MAX_SPEED = 3
        leftSpeed = max(min(leftSpeed,MAX_SPEED),-MAX_SPEED)
        rightSpeed = max(min(rightSpeed,MAX_SPEED),-MAX_SPEED)
        
        
        self.leftMotor.setVelocity(leftSpeed)
        self.rightMotor.setVelocity(rightSpeed)
        
        if (rho<0.4):
            print("Reached ", self.index+ 1,len(self.WP))
            self.index = self.index+1
            if self.index == (len(self.WP)):
                self.feedback_message = "Last waypoint reached"
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.RUNNING
                
    def terminate(self, new_status):
        """Called when the behavior stops executing."""
        self.logger.debug("%s.terminate(%s)" % (self.name, new_status))