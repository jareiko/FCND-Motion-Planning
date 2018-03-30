## Project: 3D Motion Planning

# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes...

#### `motion_planning.py`
Function | Purpose
--- | --- | --- | ---
`__main__` | parse arguments and connect to simulator
`MotionPlanning` class | encapsulate code for planning and controlling the drone
`mp.init` | set default states and register callbacks
`mp.local_position_callback` | trigger transitions due to changes in drone position
`mp.velocity_callback` | trigger disarm transition when drone comes to rest
`mp.state_callback` | trigger transitions due to state changes
`mp.arming_transition` | arm and take control of the drone
`mp.takeoff_transition` | command the drone to take off (climb into the air)
`mp.waypoint_transition` | update target to next waypoint
`mp.landing_transition` | command drone to land
`mp.disarming_transition` | command drone to disarm, and release control of it
`mp.manual_transition` | command drone to stop and go to manual control
`mp.send_waypoints` | send a dump of all waypoints to the drone for visualization
`mp.plan_path` | read in the colliders.csv file, compute a 2D grid for the target altitude using that data, compute a path from the center of the grid to [+10, +10], and send the series of waypoints to the drone

#### `planning_utils.py`
Function | Purpose
--- | --- | --- | ---
`create_grid` | compute a feasible state grid for a single given altitude, collection of colliders and a safety distance
`Action` | represents a movement the drone can make and its associated cost (presumably a combination of time and energy)
`valid_actions` | returns a list of possible `Action`s from a given position in a given grid, currently only in the four cardinal directions
`a_star` | perform the A* search algorithm to find the shortest path from start to goal in the given create_grid
`heuristic` | an estimate of the total cost from position to goal_position (must never over-estimate to be admissible as an heuristic for the A* algorithm)

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
I used a regex to parse the lat0 and lon0 from the first line of the file.
Then I added an assertion that the second line matches a defined string, and passed the rest of the file to np.loadtxt, removing the "skiprows" argument.

#### 2. Set your current local position
I used global_to_local to convert the current position to local coordinates.

#### 3. Set grid start position from local position
Round the local coordinates to integers, and offset by north_offset and east_offset.

#### 4. Set grid goal position from geodetic coords
Same as steps 2 & 3 but for the goal position.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
I've just added diagonal motion to the search. No extra creativity I'm afraid!
I used an estimated cost less than the true cost of sqrt(2) to encourage searching diagonal paths first, to favor easier-to-optimize paths with more straight line segments.

#### 6. Cull waypoints
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.



### Execute the flight
#### 1. Does it work?
It works! Successfully navigates paths around buildings, without any co-linear waypoints.

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.

# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.
