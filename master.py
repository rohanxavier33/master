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
from grab import Grab
from rotate import Rotate
from get_relative_error import GetRelativeError
from straight_forward import StraightForward
from back_up import BackUp
from let_go import LetGo

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

recog_colors = {
    'jam_jar': [0.55, 0.06, 0.06],
    'honey_jar': [0.0, 0.0, 1.0],
    'table': [1.0, 0.5, 0.0],
}


center_thresh = {
    'jam_jar': 0.005,
    'honey_jar': 0.005,
    'table': 0.05,
}
# Initialize shared data storage and populate initial values        
blackboard = Blackboard()
blackboard.write('robot', robot)  # Share robot controller instance
blackboard.write('waypoints', [(0.582, -0.02), (0.582, 0.02), (0.582, 0.04)])
blackboard.write('recognition_colors', recog_colors),
blackboard.write('centering_threshold', center_thresh),




# Build behavior tree structure
tree = Sequence("Main", children=[
    Safe("moving to safe position", blackboard),
    ReadyToGrab("moving to grabbing position", blackboard),
    Planning("moving to counter", blackboard, (.45, 0.07)),
    Navigation("move to counter", blackboard),
    Parallel(
        "Adjust position to jar",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        children=[
            GetRelativeError("Measure relative position", blackboard, 'jam_jar'),
            Rotate("Searching object", blackboard, 'jam_jar'),
        ]
    ),
    Parallel(
        "Adjust position to jar",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        children=[
            GetRelativeError("Measure relative position", blackboard, 'jam_jar'),
            StraightForward("Move forward", blackboard),
        ]
    ),
    Grab("Grab jar", blackboard),
    BackUp("Back up", blackboard),
    Safe("moving to safe position", blackboard),
    Planning("moving to table", blackboard, (0.55, -0.6)),
    Navigation("move to table", blackboard),
    Parallel(
        "Recognize Table", policy=py_trees.common.ParallelPolicy.SuccessOnOne(), children=[
            GetRelativeError("Measure relative position", blackboard, 'table'),
            Rotate("Searching table", blackboard, 'table'),
        ]
    ),
    Parallel(
        "Approach table", policy=py_trees.common.ParallelPolicy.SuccessOnOne(), children=[
            GetRelativeError("Measure relative position", blackboard, 'table'),
            StraightForward("Move forward", blackboard),
        ]
    ),
    LetGo("let go", blackboard),
    Safe("moving to safe position", blackboard),
    Planning("moving to counter", blackboard, (.45, 0.07)),
    Navigation("move to counter", blackboard),
    ReadyToGrab("moving to grabbing position", blackboard),
    Parallel(
        "Adjust position to jar",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        children=[
            GetRelativeError("Measure relative position", blackboard, 'jam_jar'),
            Rotate("Searching object", blackboard, 'jam_jar'),
        ]
    ),
    Parallel(
        "Adjust position to jar",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        children=[
            GetRelativeError("Measure relative position", blackboard, 'jam_jar'),
            StraightForward("Move forward", blackboard),
        ]
    ),
    Grab("Grab jar", blackboard),
    BackUp("Back up", blackboard),
    Safe("moving to safe position", blackboard),
    Planning("moving to table", blackboard, (0.55, -0.6)),
    Navigation("move to table", blackboard),
    Parallel(
        "Recognize Table", policy=py_trees.common.ParallelPolicy.SuccessOnOne(), children=[
            GetRelativeError("Measure relative position", blackboard, 'table'),
            Rotate("Searching table", blackboard, 'table'),
        ]
    ),
    Parallel(
        "Approach table", policy=py_trees.common.ParallelPolicy.SuccessOnOne(), children=[
            GetRelativeError("Measure relative position", blackboard, 'table'),
            StraightForward("Move forward", blackboard),
        ]
    ),
    LetGo("let go", blackboard),
    Safe("moving to safe position", blackboard),
    Planning("moving to counter", blackboard, (.45, 0.07)),
    Navigation("move to counter", blackboard),
    Stopping("stop robot", blackboard),
    ], memory=True)


# Initialize behavior tree
tree.setup_with_descendants()

# Main simulation loop
while robot.step(timestep) != -1:
    tree.tick_once()  # Execute one tick of the behavior tree