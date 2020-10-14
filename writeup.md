## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drones start location corresponds to [0, 0, 0, 0].
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

##### Test that motion_planning.py is a modified version of backyard_flyer_solution.py for simple path planning. Verify that both scripts work. Then, compare them side by side and describe in words how each of the modifications implemented in motion_planning.py is functioning.

- motion_planning.py is a modified version of backyard_flyer_solution.py: They both include the same 3 building blocks
	- A subclass of the Drone class of [UdaciDrone API](https://udacity.github.io/udacidrone/docs/welcome.html) called MotionPlanning and BackyardFlyer respectively.
	- An enum of the STATES each Drone will transition through during its missions.
	- A "__ main __" block for when each script is actually ran, which establishes a Connection to the simulated drone, creates an instance of the defined Drone subclass using that connection and starts it.

- Both scripts work 
	- MotionPlanning sets the drone in a predefined 'staircase' mission plan (sequence of waypoints).
	- BackyardFlyer sets the drone in a predefined square mission plan.
	
- __States Enum__
	- MotionPlanninng includes an additional PLANNING state in its States enum and uses automatically assigned integer codes for its states.
- __Drone Subclass__
	- MotionPlanninng imports utility functions from the provided planning_utils module, which will be used for path planning purposes.
	- The same callback functions are registered (position, velocity, state). Position and Velocity callbacks are virtually identical (position callback in MotionPlanning simply doesnt calculate the box that forms the flight plan of the BackyardFlyer - calculate_box() function is therefore missing).
	- State callback in MotionPlanning considers the PLANNING state by calling the plan_path() function when in ARMING state (and when the drone is armed) and transitioning to TAKEOFF when in PLANNING (instead of ARMING).
	- The same state transition methods are defined, and perform the same tasks except for setting home and global positions which are left for the plan_path() method in MotionPlanninng.
		
- __main block__
	- motion_planning.py uses argparse.ArgumentParser() to allow the user to specify connection parameters (host IP address and port number).
	
##### Have a look at the code, particularly in the plan_path() method and functions provided in planning_utils.py and describe what's going on there.

- __plan_path()__ is the method were I will implement my path planning solution, including:
	- Setting HOME position
	- Retrieving current GLOBAL position and converting it into the current LOCAL position
    - Finding a path from start to goal using A*
	- Prune this path to minimise the number of waypoints
	
- __planning_utils.py__ module includes functions that I will use for my path planning solution. These are based on the lecture exercises (so I have already implemented these before, one way or another) 
	- create_grid(data, drone_altitude, safety_distance): Returns a grid representation of a 2D configuration space based on given obstacle data, drone altitude and safety distance arguments.
    - a_star(grid, h, start, goal): Finds a path from start to goal conducting A* search on the grid with h as the heuristic function. This function has the following preriquisites
		- class Action(Enum): An enumeration of all possible actions at each state
		- valid_actions(grid, current_node): A function that determines which of the Actions are valid for a given current_node (e.g. the Drone cannot move NORTH if there's an obstacle to the North of the current node).

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

- I read the first line of colliders.csv using Python's csv module and extracted lat0 and lon0 as floating point values
- I set global home position to this latitude and longitude using self.set_home_position() 

And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

- I retrieved the global position using the self.global_position @Property
- I converted that to local position using global_to_local() utility function

Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

- I overrode the pre-defined grid_start and assigned it the local position calculated in the previous step
- I converted the local position into grid coordinates using the grids north/east offsets and ceiling down to ensure integer valuess, as follows: 
grid_start = (int(np.ceil(current_north-north_offset)), int(np.ceil(current_east-east_offset)))

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

- I added new optional arguments to the __main block__ so that the user can optionally provide desired geodetic coordinates for the goal position.
- If provided, this desired global position is read, converted to a local position and then to a grid position and is set as the grid_goal
- I also modified the default goal position, as it was inside a building and the drone could not land :)

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

- In planning_utils() I added  diagonal motions (NE, NW, SE, SW) with a cost of sqrt(2) to the Actions enum.
- I also modified valid_actions() method to check the validity (off-the-grid or obstacles) of diagonal moves.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

- I added a method prune_path() and use it to prune the path calculated by A* using my prune_path() method. 
- The method checks each triplet of consecutive waypoints in the original path and deletes the middle waypoint point if the triplet is colinear.  


### Execute the flight
#### 1. Does it work?
It works!

__Notes__
- The drone starts inside a building, therefore we need to fly it outside and land it before we start the motion planning script. 
- When the goal location is specified to be too far away an ![error](https://knowledge.udacity.com/questions/12507) occurs in the provided drone.py. I did not modify the provided code as my planner works fine for goals that are situated 1-2 blocks away from the start position.

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.



