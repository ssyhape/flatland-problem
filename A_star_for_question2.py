import numpy as np
from lib_piglet.utils.tools import eprint
import glob, os, sys
import copy
from queue import PriorityQueue
import time
import heapq
#import necessary modules that this python scripts need.
try:
    from flatland.core.transition_map import GridTransitionMap
    from flatland.utils.controller import get_action, Train_Actions, Directions, check_conflict, path_controller, evaluator, remote_evaluator
except Exception as e:
    eprint("Cannot load flatland modules!", e)
    exit(1)

#########################
# Debugger and visualizer options
#########################

# Set these debug option to True if you want more information printed
debug = False

visualizer = False

# If you want to test on specific instance, turn test_single_instance to True and specify the level and test number
test_single_instance = False
level = 3

test = 4

class Node:
    def __init__(self,position,direction,t,f_value):
        self.position = position
        self.direction = direction
        self.t = t
        self.f = f_value
        self.g = 0
        self.parent = None

    def __lt__(self, other):
        return self.f<other.f

    def __hash__(self):
        return hash((self.position[0], self.position[1], self.direction, self.t))
    def __eq__(self, other):
        if not isinstance(other,Node):
            return False
        return self.position[0] == other.position[0] and self.position[1] == other.position[1] and self.direction == other.direction and self.t == other.t

def h_func(loc,goal):
    return abs(loc[0]-goal[0])+abs(loc[1]-goal[1])

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
def conflict_k2(t, existing_paths, loc_pre, loc_curr):
    conflict = False
    for p in existing_paths:
        if t < len(p) and p[t] == (loc_pre[0], loc_pre[1]) and p[t - 1] == (loc_curr[0], loc_curr[1]):
            conflict = True

    return conflict


def conflict_k1(t, existing_paths, loc_pre, loc_curr):
    conflict = False
    for p in existing_paths:
        if t < len(p) and p[t] == (loc_curr[0], loc_curr[1]):
            conflict = True
    return conflict
def getsuccessors(current,rail,existing_paths,goal,w):
    successors = []
    valid_transitions = rail.get_transitions(current.position[0], current.position[1], current.direction)
    loc_pre = current.position
    t_now = current.t + 1
    if conflict_k1(t_now,existing_paths,loc_pre,loc_pre) or conflict_k2(t_now,existing_paths,loc_pre,loc_pre):
        pass
    else:
        successors.append(Node(loc_pre,current.direction,t_now,t_now + w*h_func(loc_pre,goal)))

    for i in range(len(valid_transitions)):
        loc_curr = loc_compute(loc_pre,i)
        if valid_transitions[i]==1:

            if conflict_k1(t_now, existing_paths, loc_pre, loc_curr) or conflict_k2(t_now, existing_paths, loc_pre,
                                                                                   loc_curr):
                pass
            else :
                successors.append(Node(loc_curr,i,t_now,t_now+w*h_func(loc_curr,goal)))



    return successors


def get_path2(start: tuple, start_direction: int, goal: tuple, rail: GridTransitionMap, agent_id: int,
             existing_paths: list, max_timestep: int):
    openlist = []
    g_score = {}
    closedic = {}

    w = 1
    start_node = Node(start,start_direction,0,w*h_func(start,goal))
    heapq.heappush(openlist,(start_node.f,start_node))
    g_score[start_node] = start_node
    while g_score:
        _, current = heapq.heappop(openlist)
        del g_score[current]
        closedic[current] = current
        if current.position == goal:
            path =[]
            while current:
                path.append(current.position)
                current = current.parent
            path = path[::-1]
            return path

        for s in getsuccessors(current,rail,existing_paths,goal,w):
            if s  in closedic:
                continue
            s.g = current.g + 1
            s.f = s.g + w*h_func(s.position,goal)
            s.parent = current

            if s in g_score:
                pre_s = g_score[s]

                if s.g<pre_s.g:
                    pre_s.g = s.g
                    pre_s.f = s.f
                    pre_s.parent =s.parent
                    heapq.heapify(openlist)

            else:
                heapq.heappush(openlist,(s.f,s))
                g_score[s] = s




    return []



if __name__ == "__main__":
    start_time = time.time()
    if len(sys.argv) > 1:
        remote_evaluator(get_path2,sys.argv)
    else:
        script_path = os.path.dirname(os.path.abspath(__file__))
        test_cases = glob.glob(os.path.join(script_path,"multi_test_case/level*_test_*.pkl"))
        if test_single_instance:
            test_cases = glob.glob(os.path.join(script_path,"multi_test_case/level{}_test_{}.pkl".format(level, test)))
        test_cases.sort()
        evaluator(get_path2,test_cases,debug,visualizer,2)
    end_time = time.time()

    print("total_time:",end_time-start_time)