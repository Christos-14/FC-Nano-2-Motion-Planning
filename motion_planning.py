import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import csv

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, goal=None):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # This is added to accomplish the flexible goal position requirement
        # If a goal location is specified, it is going to be interpeted as a gloabal position (lon, lat, alt) and used instead of the default
        self.goal = None
        if goal and len(goal)==3 and goal[0] and goal[1] and goal[2]:
            self.goal = goal

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    # Method for path pruning based on waypoint colineaity
    def prune_path(self, path, colinearity_tolerance=1):
        i = 0
        while i < len(path) - 2:
            # We check each triplet of consecutive waypoints
            point_1 = np.array([path[i][0], path[i][1], 1.]).reshape(1, -1)
            point_2 = np.array([path[i+1][0], path[i+1][1], 1.]).reshape(1, -1)
            point_3 = np.array([path[i+2][0], path[i+2][1], 1.]).reshape(1, -1)
            # If the waypoints are (nearly) colinear, we remove the middle one from the path
            if abs(np.linalg.det(np.concatenate((point_1, point_2, point_3), 0))) < colinearity_tolerance:
                path.remove(path[i+1])
            else:
                i += 1
        return path

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # DONE: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv', newline='') as f:
            reader = csv.reader(f)
            row1 = next(reader)
            lat0, lon0 = float(row1[0][5:]), float(row1[1][5:])

        # DONE: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # DONE: retrieve current global position
        # I do this using self.global_position @Property
        
        # DONE: convert to current local position using global_to_local()
        #  Convert a global position (lon, lat, up) to a local position (north, east, down) relative to the home position.
        current_north, current_east, _ = global_to_local(self.global_position, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)
        # DONE: convert start position to current position rather than map center
        grid_start = (int(np.ceil(current_north-north_offset)), int(np.ceil(current_east-east_offset)))
        
        # Set goal as some arbitrary position on the grid
        # grid_goal = (-north_offset + 10, -east_offset + 10)
        # Changed this so that default goal position is not inside the building and the drone is able to land!
        grid_goal = (-north_offset +30 , -east_offset + 10)
        # DONE: adapt to set goal as latitude / longitude position and convert
        # If a goal location has been specified we override the default
        if self.goal:
            goal_north, goal_east, _ = global_to_local(self.goal, self.global_home)
            grid_goal = (int(goal_north - north_offset), int(goal_east - east_offset))
            print('Goal specified by user!')
            print('Goal - Global position: {}'.format(self.goal))
            print('Goal - Grid position  : {}'.format(grid_goal))

        # Run A* to find a path from start to goal
        # DONE: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        # This is done in planning_utils.py
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print('Original path length: {}'.format(len(path)))
        
        # DONE: prune path to minimize number of waypoints
        path = self.prune_path(path)
        print('Pruned path length: {}'.format(len(path)))
        
        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # DONE: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    # Adding optional arguments for specific goal position (geodetic coordinates)- If the user wants to specify a goal position, they must pass all 3 arguments
    parser.add_argument('--glon', type=float, default=None, help="Goal longitude (-122.XXXXX)")
    parser.add_argument('--glat', type=float, default=None, help="Goal latitude (37.XXXXX)")
    parser.add_argument('--galt', type=float, default=None, help="Goal altitude (Non-zero! Pass something small, like 0.05)")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    # If goal position arguments have been provided, pass them to the Drone
    if args.glon and args.glat and args.galt:
        print("Passing custom goal position to the drone...")
        drone = MotionPlanning(conn, goal=[float(args.glon), float(args.glat), float(args.galt)])
    else:
        drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
