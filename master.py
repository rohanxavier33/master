# Module Imports
# --------------------------
import numpy as np  # Numerical operations
import py_trees  # Behavior tree implementation
from py_trees.composites import Sequence, Parallel, Selector  # Behavior tree nodes

# Custom behavior implementations for robot tasks
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

# Webots robot control API
from controller import Robot, Supervisor

# --------------------------
# Robot Initialization
# --------------------------
# Create Webots Supervisor instance with default timestep
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# --------------------------
# Shared Data Storage
# --------------------------
# Simple blackboard pattern for cross-behavior data sharing
class Blackboard:
    def __init__(self):
        self.data = {}  # Key-value storage
    def write(self, key, value):
        self.data[key] = value  # Store any type of data
    def read(self, key):
        return self.data.get(key)  # Retrieve data or None

# Object recognition parameters (color signatures)
recog_colors = {
    'jam_jar1': [0.55, 0.06, 0.06],  # Dark red
    'jam_jar2': [1, 1, 1],           # White
    'honey_jar': [0.0, 0.0, 1.0],    # Blue
    'table': [1.0, 0.5, 0.0],        # Orange
}

# Alignment thresholds for different objects
center_thresh = {
    'jam_jar1': 0.005,   # Loose precision
    'jam_jar2': 0.001,   # Medium precision
    'honey_jar': 0.0005, # High precision
    'table': 0.05,       # Coarse alignment
}

# --------------------------
# Global Configuration
# --------------------------
# Initialize and populate blackboard with initial settings
blackboard = Blackboard()
blackboard.write('robot', robot)  # Share robot controller instance
blackboard.write('waypoints', [  # Navigation waypoints (x,y coordinates)
    (.75,0),(.75,-1.2),(.75,-2),(.75, -3.15),(-.84,-3.15),
    (-1.65,-3),(-1.65, -2.6),(-1.65, -2),(-1.7,-.5),
    (-1.7,0),(-.76,0.4), (0,0)
])
blackboard.write('recognition_colors', recog_colors),  # Color profiles
blackboard.write('centering_threshold', center_thresh),  # Alignment tolerances

# --------------------------
# Behavior Tree Construction
# --------------------------
# Main sequence defines the robot's high-level workflow

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
            GetRelativeError("Measure relative position", blackboard, 'jam_jar1'),
            Rotate("Searching object", blackboard, 'jam_jar1'),
        ]
    ),
    Parallel(
        "Adjust position to jar",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        children=[
            GetRelativeError("Measure relative position", blackboard, 'jam_jar1'),
            StraightForward("Move forward", blackboard),
        ]
    ),
    Grab("Grab jar", blackboard),
    BackUp("Back up", blackboard),
    Safe("moving to safe position", blackboard),
    Planning("moving to table", blackboard, (0.4, -0.6)),
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
    Planning("moving to counter", blackboard, (.14, 0.08)),
    Navigation("move to counter", blackboard),
    Parallel(
        "Adjust position to jar",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        children=[
            GetRelativeError("Measure relative position", blackboard, 'jam_jar2'),
            Rotate("Searching object", blackboard, 'jam_jar2'),
        ]
    ),
    ReadyToGrab("moving to grabbing position", blackboard),
    Parallel(
        "Adjust position to jar",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        children=[
            GetRelativeError("Measure relative position", blackboard, 'jam_jar2'),
            StraightForward("Move forward", blackboard),
        ]
    ),
    Grab("Grab jar", blackboard),
    BackUp("Back up", blackboard),
    Safe("moving to safe position", blackboard),
    Planning("moving to table", blackboard, (0.5, -0.5)),
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
    Planning("moving to counter", blackboard, (.14, 0.08)),
    Navigation("move to counter", blackboard),
    Parallel(
        "Adjust position to jar",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        children=[
            GetRelativeError("Measure relative position", blackboard, 'honey_jar'),
            Rotate("Searching object", blackboard, 'honey_jar'),
        ]
    ),
    ReadyToGrab("moving to grabbing position", blackboard),
    Parallel(
        "Adjust position to jar",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        children=[
            GetRelativeError("Measure relative position", blackboard, 'honey_jar'),
            StraightForward("Move forward", blackboard),
        ]
    ),
    Grab("Grab jar", blackboard),
    BackUp("Back up", blackboard),
    Safe("moving to safe position", blackboard),
    Planning("moving to table", blackboard, (0.45, -0.55)),
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
    BackUp("Back up", blackboard),
    Safe("moving to safe position", blackboard),
    Stopping("stop robot", blackboard),
    ], memory=True)


# Initialize behavior tree
tree.setup_with_descendants()

# Main simulation loop
while robot.step(timestep) != -1:
    tree.tick_once()  # Execute one tick of the behavior tree