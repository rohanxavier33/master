"""planning controller."""

import py_trees
import numpy as np
from collections import defaultdict
from heapq import heapify, heappush, heappop
import os # Import os to check file existence easily

# Function to convert world coordinates to map pixel coordinates
def world2map(xw, yw):
    """Converts world coordinates (xw, yw) to map pixel coordinates (px, py).

    Args:
        xw: World x-coordinate.
        yw: World y-coordinate.

    Returns:
        A list containing the [x_pixel, y_pixel] map coordinates.
    """
    # World coordinate bounds (adjust if your world is different)
    x_min, x_max = -2.25, 1.8
    y_min, y_max = -4.0, 1.8

    # Desired center in world coordinates (adjust based on your map's focus)
    center_x = -0.7
    center_y = -1.5

    # Map dimensions (must match the Display node in Webots)
    # Assuming 300x300 based on mapping code drawPixel calls
    map_width_pixels = 300
    map_height_pixels = 300

    # Calculate maximum distances from the center to the world edges
    max_x_distance = max(abs(x_min - center_x), abs(x_max - center_x)) # Should be 2.5
    max_y_distance = max(abs(y_max - center_y), abs(y_min - center_y)) # Should be 3.3 (check calculation: max(1.8 - (-1.5), -4.0 - (-1.5)) = max(3.3, -2.5) -> 3.3)


    # Scaling factors (pixels per world unit) - ensure map is centered
    # Use half the map dimension for the distance scaling
    scale_x = (map_width_pixels / 2) / max_x_distance
    scale_y = (map_height_pixels / 2) / max_y_distance

    # Convert x coordinate
    # Add half map width to shift origin to the center
    x_pixel = (xw - center_x) * scale_x + (map_width_pixels / 2)
    # Convert y coordinate (inverted y-axis for typical image coordinates)
    # Add half map height to shift origin to the center
    y_pixel = (map_height_pixels / 2) - (yw - center_y) * scale_y

    # Clamp and round to ensure within map bounds (0 to width/height - 1)
    x_pixel = int(round(max(0, min(map_width_pixels - 1, x_pixel))))
    y_pixel = int(round(max(0, min(map_height_pixels - 1, y_pixel))))

    return [x_pixel, y_pixel]

# Function to convert map pixel coordinates back to world coordinates
def map2world(px, py):
    """Converts map pixel coordinates (px, py) back to world coordinates (xw, yw).

    Args:
        px: Map x-pixel coordinate.
        py: Map y-pixel coordinate.

    Returns:
        A list containing the [xw, yw] world coordinates.
    """
    # --- Must match the parameters used in world2map ---
    x_min, x_max = -2.25, 1.8
    y_min, y_max = -4.0, 1.8
    center_x = -0.7
    center_y = -1.5
    map_width_pixels = 300
    map_height_pixels = 300
    max_x_distance = max(abs(x_min - center_x), abs(x_max - center_x)) # 2.5
    max_y_distance = max(abs(y_max - center_y), abs(y_min - center_y)) # 3.3
    scale_x = (map_width_pixels / 2) / max_x_distance
    scale_y = (map_height_pixels / 2) / max_y_distance
    # --- End of matching parameters ---

    # Reverse the conversion process
    xw = ((px - (map_width_pixels / 2)) / scale_x) + center_x
    # Corrected reversal for y
    yw = center_y - ((py - (map_height_pixels / 2)) / scale_y)


    return [xw, yw]


# A* algorithm implementation to find the shortest path in the configuration space
def getShortestPath(map_data, start, goal):
    """Calculates the shortest path using A* algorithm.

    Args:
        map_data: The configuration space numpy array (0 for free, >0 for obstacle).
        start: Tuple (px, py) representing the starting pixel coordinates.
        goal: Tuple (px, py) representing the goal pixel coordinates.

    Returns:
        A list of tuples representing the path in pixel coordinates [(px1, py1), ...],
        or an empty list if no path is found.
    """
    map_rows, map_cols = map_data.shape

    # Heuristic function (Euclidean distance) for A*
    def heuristic(a, b):
      return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    # Function to get valid neighbors of a node (pixel)
    def getNeighbors(v):
        """Gets valid neighboring pixels for path planning."""
        neighbors=[]
        # Define 8 possible movements (including diagonals)
        for delta in ((-1,0), (-1,-1), (0,-1), (+1,-1), (+1,0), (+1,+1), (0,+1), (-1,+1)):
            u = (v[0] + delta[0], v[1] + delta[1]) # Calculate neighbor coordinates

            # Check if the neighbor is within map boundaries
            # Corrected map boundary check using shape
            if 0 <= u[0] < map_rows and 0 <= u[1] < map_cols:
                 # Check if the neighbor is in free space (value close to 0 in cspace)
                 # Use a small threshold to account for potential floating point inaccuracies
                 # Increased threshold slightly, as convolution might not yield exact 0
                if map_data[u[0], u[1]] < 0.5: # Check if it's free space in the configuration space
                    # Calculate cost (Euclidean distance)
                    cost = np.sqrt(delta[0]**2 + delta[1]**2)
                    neighbors.append((u, cost))
        return neighbors

    # --- A* Algorithm Implementation ---
    # Priority queue storing (f_score, node)
    # f_score = g_score + heuristic
    queue = [(0 + heuristic(start, goal), 0, start)] # (f_score, g_score, node)
    heapify(queue) # Turn the list into a min-heap

    # Keep track of the path taken to reach each node
    parent = {start: None}
    # Store the cost (g_score) to reach each node from the start
    g_scores = defaultdict(lambda: float('inf'))
    g_scores[start] = 0

    visited_for_path = set() # Keep track of nodes already processed

    while queue:
        # Get the node with the lowest f_score
        f_score, current_g, u = heappop(queue)

        # Optimization: If current f_score is worse than stored g_score, skip
        # (Handles duplicates in the queue with potentially worse paths)
        if f_score > g_scores[u] + heuristic(u, goal): # Check if f_score based on current_g is worse
             continue

        # If we reached the goal, reconstruct and return the path
        if u == goal:
            path = []
            curr = goal
            while curr is not None:
                path.append(curr)
                curr = parent.get(curr) # Use .get for safety
            return path[::-1] # Return reversed path (start to goal)

        # Avoid reprocessing nodes (alternative placement, can be here or after pop)
        # if u in visited_for_path:
        #    continue
        # visited_for_path.add(u)


        # Explore neighbors
        for v, costuv in getNeighbors(u):
            # Calculate tentative g_score for the neighbor
            tentative_g_score = g_scores[u] + costuv

            # If this path to the neighbor is better than any previous path found so far
            if tentative_g_score < g_scores[v]:
                # Update parent, g_score, and f_score
                parent[v] = u
                g_scores[v] = tentative_g_score
                f_score_v = tentative_g_score + heuristic(v, goal)
                # Add neighbor to the priority queue
                heappush(queue, (f_score_v, tentative_g_score, v)) # Push (f_score, g_score, node)

    # If the loop finishes without finding the goal, return empty path
    print(f"Planning: No path found from {start} to {goal}")
    return []


# Behavior Tree node for Planning
class Planning(py_trees.behaviour.Behaviour):
    """
    A py_trees behavior node responsible for loading the map,
    calculating a path from the robot's current location to a specified goal,
    and writing the path (as waypoints) to the blackboard.
    """
    def __init__(self, name, blackboard, goal_world_coords):
        """
        Initializes the Planning behavior.

        Args:
            name (str): The name of the behavior node.
            blackboard (Blackboard): The shared blackboard object.
            goal_world_coords (tuple): The target goal (xw, yw) in world coordinates.
        """
        super(Planning, self).__init__(name)
        self.blackboard = blackboard
        self.robot = blackboard.read('robot') # Read robot instance from blackboard

        # Convert world goal coordinates to map pixel coordinates and store
        px, py = world2map(goal_world_coords[0], goal_world_coords[1])
        self.goal_map_coords = (px, py) # Store goal in map coordinates (tuple for hashing)

        self.cspace = None # To store the loaded configuration space map
        self.gps = None    # To store the GPS device
        self.timestep = -1 # To store the simulation timestep
        self._path_calculated = False # Flag to indicate if path calculation was attempted

        # Add logger for debugging
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))


    def setup(self, **kwargs):
        """
        Called once before the behavior runs the first time.
        Sets up robot devices. Map loading moved to initialise.
        """
        self.logger.debug("%s.setup()->setting up GPS" % self.name)
        try:
            # Get simulation timestep
            self.timestep = int(self.robot.getBasicTimeStep())

            # Get and enable GPS device
            self.gps = self.robot.getDevice('gps')
            self.gps.enable(self.timestep)
            # NOTE: Map loading is deferred to initialise()

        except Exception as e:
            self.logger.error("%s: Failed to setup GPS: %s" % (self.name, e))
            # We might want to handle this more robustly, but for now,
            # initialise will likely fail if GPS is needed and not available.

    def initialise(self):
        """
        Called once when the behavior starts executing.
        Loads the map, calculates the path, and writes it to the blackboard.
        """
        self.logger.debug("%s.initialise()->loading map and calculating path" % self.name)
        self._path_calculated = False # Reset flag for this run

        # --- Load Map ---
        map_file = 'cspace.npy'
        if not os.path.exists(map_file):
            self.logger.error(f"{self.name}: Map file '{map_file}' not found. Cannot plan.")
            self.cspace = None
            # No need to proceed with planning if map doesn't exist
            return
        try:
            self.cspace = np.load(map_file)
            self.logger.info(f"{self.name}: Configuration space map '{map_file}' loaded successfully.")
        except Exception as e:
            self.logger.error(f"{self.name}: Failed to load map '{map_file}': {e}")
            self.cspace = None
             # No need to proceed with planning if map loading failed
            return

        # --- Calculate Path ---
        # Check if GPS is ready (might take a step)
        if not self.gps:
             self.logger.error(f"{self.name}: GPS not available. Cannot get current position.")
             return
        self.robot.step(self.timestep) # Ensure GPS value is current
        gps_values = self.gps.getValues()
        if len(gps_values) < 2:
             self.logger.error(f"{self.name}: Invalid GPS values received: {gps_values}")
             return

        xw = gps_values[0]
        yw = gps_values[1]

        # Convert current world coordinates to map pixel coordinates
        try:
             start_px, start_py = world2map(xw, yw)
             start_map_coords = (start_px, start_py) # Use tuple for consistency
        except Exception as e:
             self.logger.error(f"{self.name}: Error converting start position ({xw}, {yw}) to map coords: {e}")
             return

        # Ensure start and goal are within map bounds before planning
        rows, cols = self.cspace.shape
        if not (0 <= start_map_coords[0] < rows and 0 <= start_map_coords[1] < cols):
            self.logger.error(f"{self.name}: Start position {start_map_coords} is outside map bounds ({rows}x{cols}). Cannot plan.")
            return
        if not (0 <= self.goal_map_coords[0] < rows and 0 <= self.goal_map_coords[1] < cols):
             self.logger.error(f"{self.name}: Goal position {self.goal_map_coords} is outside map bounds ({rows}x{cols}). Cannot plan.")
             return

        # Check if start or goal is inside an obstacle in cspace
        if self.cspace[start_map_coords[0], start_map_coords[1]] > 0.5: # Using threshold from getNeighbors
             self.logger.warning(f"{self.name}: Start position {start_map_coords} is in an obstacle area. Planning might fail.")
             # Optionally return here or let getShortestPath handle it
        if self.cspace[self.goal_map_coords[0], self.goal_map_coords[1]] > 0.5:
             self.logger.warning(f"{self.name}: Goal position {self.goal_map_coords} is in an obstacle area. Planning might fail.")
             # Optionally return here or let getShortestPath handle it


        self.logger.info(f"{self.name}: Planning path from {start_map_coords} world({xw:.2f},{yw:.2f}) to {self.goal_map_coords}")

        # Calculate the shortest path using the loaded map and A*
        path_map_coords = getShortestPath(self.cspace, start_map_coords, self.goal_map_coords)
        self._path_calculated = True # Mark that calculation was attempted

        if path_map_coords:
            self.logger.info(f"{self.name}: Path found with {len(path_map_coords)} points.")
            # Convert path from map coordinates back to world coordinates
            path_world_coords = []
            # Simplify path - Take every Nth point + start/end
            step = 10 # Adjust simplification level
            indices_to_keep = list(range(0, len(path_map_coords), step))
            # Ensure the last point is always included if not already
            if len(path_map_coords) > 0 and (len(path_map_coords) - 1) not in indices_to_keep:
                 indices_to_keep.append(len(path_map_coords) - 1)

            for i in indices_to_keep:
                 px, py = path_map_coords[i]
                 xw_path, yw_path = map2world(px, py)
                 path_world_coords.append((xw_path, yw_path))

            # Ensure the list isn't empty after simplification (should at least have goal)
            if not path_world_coords and path_map_coords:
                 px_goal, py_goal = self.goal_map_coords
                 xw_goal, yw_goal = map2world(px_goal, py_goal)
                 path_world_coords.append((xw_goal, yw_goal))

            self.logger.info(f"{self.name}: Simplified path to {len(path_world_coords)} waypoints.")

            # Write the calculated world coordinate path to the blackboard for Navigation behavior
            self.blackboard.write('waypoints', np.array(path_world_coords))
            # self.logger.debug(f"{self.name}: Waypoints written to blackboard: {path_world_coords}") # Can be verbose
        else:
            self.logger.warning("%s: No path found. Waypoints will not be updated." % self.name)
            # Explicitly write an empty array to signal failure to Navigation
            self.blackboard.write('waypoints', np.array([]))


    def update(self):
        """
        Called every tick while the behavior is running.
        Planning is done in initialise, so this just returns SUCCESS or FAILURE.

        Returns:
            py_trees.common.Status: SUCCESS if planning was successful (path found and written),
                                    FAILURE otherwise (map load fail, no path found, etc.).
        """
        # Check if map loading failed or path calculation wasn't even attempted
        if self.cspace is None or not self._path_calculated:
             self.logger.debug("%s.update() -> FAILURE (Map not loaded or planning skipped)" % self.name)
             return py_trees.common.Status.FAILURE

        # Check if a valid path was found and written to the blackboard
        waypoints = self.blackboard.read('waypoints') # Read waypoints written in initialise
        if waypoints is not None and len(waypoints) > 0:
            self.logger.debug("%s.update() -> SUCCESS (Planning complete, path found)" % self.name)
            return py_trees.common.Status.SUCCESS # Planning is done in one go
        else:
            # This case handles when getShortestPath returned an empty list OR waypoints are explicitly empty
            self.logger.debug("%s.update() -> FAILURE (No path found or waypoints empty)" % self.name)
            return py_trees.common.Status.FAILURE


    def terminate(self, new_status):
        """
        Called when the behavior stops executing.

        Args:
            new_status (py_trees.common.Status): The status indicating why it stopped.
        """
        self.logger.debug("%s.terminate(%s)" % (self.name, new_status))
        # Reset flag and clear loaded map potentially
        self._path_calculated = False
        # self.cspace = None # Optional: Clear map if memory is a concern
        # No specific cleanup needed for planning in this case