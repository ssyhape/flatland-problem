"""
This is the python script for question 1. In this script, you are required to implement a single agent path-finding algorithm
"""
from lib_piglet.utils.tools import eprint
import glob, os, sys
from queue import PriorityQueue
import copy
#import necessary modules that this python scripts need.
try:
    from flatland.core.transition_map import GridTransitionMap
    from flatland.utils.controller import get_action, Train_Actions, Directions, check_conflict, path_controller, evaluator, remote_evaluator
except Exception as e:
    eprint("Cannot load flatland modules!")
    eprint(e)
    exit(1)

#########################
# Debugger and visualizer options
#########################

# Set these debug option to True if you want more information printed
debug = False
visualizer = False

# If you want to test on specific instance, turn test_single_instance to True and specify the level and test number
test_single_instance = False
level = 0
test = 0


#########################
# Reimplementing the content in get_path() function.
#
# Return a list of (x,y) location tuples which connect the start and goal locations.
#########################


# This function return a list of location tuple as the solution.
# @param start A tuple of (x,y) coordinates
# @param start_direction An Int indicate direction.
# @param goal A tuple of (x,y) coordinates
# @param rail The flatland railway GridTransitionMap
# @param max_timestep The max timestep of this episode.
# @return path A list of (x,y) tuple.


class Node_for_Astar:
    """
    loc : current location
    direc : The current direction of agent
    level : the p_value
    value : f_value
    path : The  shortest path to the state
    """
    def __init__(self, loc, direction, value, level,path):
        self.direc = direction
        self.level = level
        self.f_value = value
        self.loc = loc
        self.path = path

    def __lt__(self, other):
        return self.f_value < other.f_value


def h_func(current_state, goal):
    """
    compute h_value
    """
    return abs(current_state[0] - goal[0]) + abs(current_state[1] - goal[1])


def loc_compute(loc_curr, action):
    if action == Directions.NORTH:
        return tuple((loc_curr[0] - 1, loc_curr[1]))
    elif action == Directions.EAST:
        return tuple((loc_curr[0], loc_curr[1] + 1))
    elif action == Directions.SOUTH:
        return tuple((loc_curr[0] + 1, loc_curr[1]))
    elif action == Directions.WEST:
        return tuple((loc_curr[0], loc_curr[1] - 1))
    else:
        return 'error'


def get_path(start: tuple, start_direction: int, goal: tuple, rail: GridTransitionMap, max_timestep: int):
    path = []
    loc = start
    direction = start_direction
    queue_of_loc = PriorityQueue()
    path.append(start)
    queue_of_loc.put(Node_for_Astar(loc, direction, 0, 0, path))
    while (queue_of_loc.empty() != 1):
        tmp = queue_of_loc.get()
        if tmp.loc == goal:
            path = tmp.path
            break
        valid_transitions = rail.get_transitions(tmp.loc[0], tmp.loc[1], tmp.direc)
        level_curreent = tmp.level + 1
        curr_path = tmp.path
        for i in range(0, len(valid_transitions)):
            if valid_transitions[i]:
                loc_curr = loc_compute(tmp.loc, i)
                f_value = level_curreent + h_func(loc_curr, goal)
                tmp_path = copy.deepcopy(curr_path)
                tmp_path.append(loc_curr)
                tmp_node = Node_for_Astar(loc_curr, i, f_value, level_curreent,tmp_path)
                queue_of_loc.put(tmp_node)

    time_limit = max_timestep / 10
    if len(path) > time_limit:
        return path[0:time_limit]
    else:
        return path


#########################
# You should not modify codes below, unless you want to modify test_cases to test specific instance. You can read it know how we ran flatland environment.
########################
if __name__ == "__main__":
    if len(sys.argv) > 1:
        remote_evaluator(get_path,sys.argv)
    else:
        script_path = os.path.dirname(os.path.abspath(__file__))
        test_cases = glob.glob(os.path.join(script_path,"single_test_case/level*_test_*.pkl"))
        if test_single_instance:
            test_cases = glob.glob(os.path.join(script_path,"single_test_case/level{}_test_{}.pkl".format(level, test)))
        test_cases.sort()
        # Use the additionally written get_path2() here as the A star algorithm for detection.
        evaluator(get_path,test_cases,debug,visualizer,1)



















