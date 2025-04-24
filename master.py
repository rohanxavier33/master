from os.path import exists
import numpy as np
import py_trees
from py_trees.composites import Sequence, Parallel, Selector

# Import custom behavior modules
from safe import Safe
from ready_to_grab import ReadyToGrab
from navigation import Navigation
from planning import Planning
from stopping import Stopping

from controller import Robot, Supervisor

# Initialize Webots Supervisor robot
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Simple key-value store for sharing data between behaviors            
class Blackboard:
    def __init__(self):
        self.data = {}
    def write(self, key, value):
        self.data[key] = value
    def read(self, key):
        return self.data.get(key)


# Initialize shared data storage and populate initial values        
blackboard = Blackboard()
blackboard.write('robot', robot)  # Share robot controller instance
blackboard.write('waypoints', [(0.582, -0.02), (0.582, 0.02), (0.582, 0.04)])  # Example waypoints


# Build behavior tree structure
tree = Sequence("Main", children=[
    Safe("moving to safe position", blackboard),
    ReadyToGrab("moving to grabbing position", blackboard),
    Planning("moving to counter", blackboard, (.7, -0.06)),
    Navigation("move to counter", blackboard),
    Stopping("stop robot", blackboard),
    ], memory=True)


# Initialize behavior tree
tree.setup_with_descendants()

# Main simulation loop
while robot.step(timestep) != -1:
    tree.tick_once()  # Execute one tick of the behavior tree