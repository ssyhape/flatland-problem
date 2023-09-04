import heapq

import numpy as np
from lib_piglet.utils.tools import eprint
import glob, os, sys
import copy
from queue import PriorityQueue
import time
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
level = 1
test = 1

class Node_for_Astar:
    """
    loc : current location
    direc : The current direction of agent
    level : the p_value
    value : f_value
    path : The  shortest path to the state
    """

    def __init__(self, loc, direction, value, level, path,mat):
        self.direc = direction
        self.level = level
        self.f_value = value
        self.loc = loc
        self.path = path
        self.vis_mat = mat

    def __lt__(self, other):
        return self.f_value < other.f_value

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


class Node_for_SIPP:
    """
    time : The time when arriving the node
    interval : safe time interval
    f_value : the value of evaluation function
    parent : parent node
    """

    def __init__(self,time,loc,direc,ini_time_bounder,value,g_value):
        self.time = time
        self.loc = loc
        self.direction = direc
        self.interval=ini_time_bounder
        self.f_value = value
        self.g_value = g_value
        self.parent = None

    def __lt__(self, other):
        return self.f_value < other.f_value

    def __hash__(self):
        return hash((self.loc,self.direction,self.interval[0],self.interval[1]))

    def __eq__(self, other):
        if not isinstance(other,Node_for_SIPP):
            return False
        return  self.direction == self.direction and self.interval == other.interval and self.loc == other.loc


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
def get_config(node,XYTmatrix,action,max_time_bounder):
    new_loc = loc_compute(node.loc,action)
    cfgs = []
    #Get the intervals list
    intervels_new_loc = []
    intervel_ind_new_loc = XYTmatrix[new_loc[0]][new_loc[1]]
    if len(intervel_ind_new_loc) == 0:
        return [(node.time+1,(0,max_time_bounder))]  # (arrive time,(safe interval))
    intervel_ind_new_loc.insert(0,0)
    for i,j in enumerate(intervel_ind_new_loc[:-1]):
        pre = i
        nex = i+1
        if intervel_ind_new_loc[nex] - intervel_ind_new_loc[pre]!=1:
            intervels_new_loc.append((max(node.time+1,intervel_ind_new_loc[pre]+1),(intervel_ind_new_loc[pre]+1,intervel_ind_new_loc[nex]-1)))

    last_time = intervel_ind_new_loc[-1]
    if last_time < max_time_bounder:
        intervels_new_loc.append((max(node.time+1,last_time+1),(last_time+1,max_time_bounder)))

    return intervels_new_loc  # return is a list of ((j1,(s1,e1)),..,(j2,(sn,en)))


def conflict(loc,t,XYTmartix):
    for tt in XYTmartix[loc[0]][loc[1]]:
        if tt == t:
            return True

    return False


def getSuccessors(node,rail,XYT_matrix,max_time_bounder):
    successors = []
    valid_transitions = rail.get_transitions(node.loc[0], node.loc[1], node.direction)
    for i in range(len(valid_transitions)):
        if valid_transitions[i] ==1:
            new_loc = loc_compute(node.loc,i)
            new_g_value = node.g_value  #uncorrect it will be modified in getpath2()
            new_f_value = new_g_value  # for convience becasue the correct computing will in the func getpath2()
            cfgs = get_config(node,XYT_matrix,i,max_time_bounder)
            start_t = node.time + 1
            end_t = node.interval[1] + 1
            for cfg in cfgs:
                if cfg[1][0]>end_t or cfg[1][1] <start_t:
                    continue
                if conflict(node.loc,cfg[0],XYT_matrix):
                    continue

                t_node = Node_for_SIPP(cfg[0],new_loc,i,[cfg[1][0],cfg[1][1]],new_f_value,new_g_value)
                successors.append(t_node)


    return successors


def A_star_normal(start: tuple, start_direction: int, goal: tuple, rail: GridTransitionMap, agent_id: int,
             existing_paths: list, max_timestep: int):
    path = []
    loc = start
    direction = start_direction
    queue_of_loc = PriorityQueue()
    path.append(start)
    vis_mat_ini = np.zeros((rail.width,rail.height))
    vis_mat_ini[loc[0]][loc[1]] = 1
    queue_of_loc.put(Node_for_Astar(loc, direction, 0, 0, path,vis_mat_ini))

    search_start_time = time.time()
    while (queue_of_loc.empty() != 1):
        tmp = queue_of_loc.get()
        path = tmp.path
        part_time = time.time()
        if part_time - search_start_time>= 100:
            return [start]
        if tmp.loc == goal:
            path = tmp.path
            break
        valid_transitions = rail.get_transitions(tmp.loc[0], tmp.loc[1], tmp.direc)
        level_current = tmp.level + 1  # represent the t
        curr_path = tmp.path
        vis_curr = tmp.vis_mat
        for i in range(0, len(valid_transitions)):
            loc_curr = loc_compute(tmp.loc, i)
            if valid_transitions[i] and vis_curr[loc_curr[0],loc_curr[1]] == 0:
                t_now = level_current
                f_value = level_current + h_func(loc_curr, goal)
                tmp_path = copy.deepcopy(curr_path)
                tmp_vis = copy.deepcopy(vis_curr)
                tmp_vis[loc_curr[0],loc_curr[1]] = 1
                tmp_path.append(loc_curr)
                if conflict_k2(level_current, existing_paths, tmp.loc,
                               loc_curr):  # conflict kind 2 (which cannot be avoided)
                    continue
                if conflict_k1(level_current, existing_paths, tmp.loc,
                               loc_curr):  # conflict kind 1 can be avoided(just wait):
                    tmp_path[-1] = tmp.loc
                    tmp_vis[loc_curr[0],loc_curr[1]] =0
                    loc_curr = tmp.loc
                    f_value = tmp.f_value
                tmp_node = Node_for_Astar(loc_curr, i, f_value, t_now, tmp_path,tmp_vis)
                queue_of_loc.put(tmp_node)

    time_limit = max_timestep
    if len(path) > time_limit:
        path = path[0:time_limit]
    if path[-1] != goal:
        return [start]
    else:
        return path
def get_ini_cfg(loc,XYT,max_timebounder):
    intervel_ind_new_loc= XYT[loc[0]][loc[1]]
    if len(intervel_ind_new_loc) ==0:
        return (0,(0,max_timebounder))
    else:
        return(0,(0,intervel_ind_new_loc[0]))


def get_path2(start: tuple, start_direction: int, goal: tuple, rail: GridTransitionMap, agent_id: int,
             existing_paths: list, max_timestep: int):
    if len(existing_paths) ==0:
        return A_star_normal(start,start_direction,goal,rail,agent_id,existing_paths,max_timestep)

    # XYT means a matrix of X*Y*listlen to save the time point of the pre paths
    XYT_mat = [[None] * rail.width for _ in range(rail.height)]
    for x in range(rail.width):
        for y in range(rail.height):
            t_ind_of_xy = []
            for p in existing_paths:
                for t in range(len(p)):
                    if p[t] == (x,y):
                        t_ind_of_xy.append(t)
            t_ind_of_xy.sort()
            XYT_mat[x][y] = t_ind_of_xy
    path_success = []
    openlist = []
    g_score = {}
    closedic = {}
    # Get initial node
    loc = start
    direction = start_direction
    max_time_bounder = max_timestep
    path_ini = [loc]
    vis_mat_ini = np.zeros((rail.width,rail.height))
    vis_mat_ini[loc[0]][loc[1]] = 1
    ini_cfgs = get_ini_cfg(loc,XYT_mat,max_time_bounder)
    s_start = Node_for_SIPP(0,loc,direction,[0,ini_cfgs[1][1]],h_func(loc,goal),0)
    heapq.heappush(openlist,(s_start.f_value,s_start))
    #Set loc,direction and safe interval as the unique mark
    g_score[s_start] = s_start
    while g_score:
        _,tmp_node = heapq.heappop(openlist)
        del g_score[tmp_node]
        closedic[tmp_node] = tmp_node
        # Get the final path
        if tmp_node.loc == goal:
            path = []
            current = tmp_node
            while current:
                path.append(current)
                current = current.parent
            path = path[::-1]
            path_success =path
            break

        #Get Successors
        successors = getSuccessors(tmp_node,rail,XYT_mat,max_time_bounder)
        for s in successors:

            s.g_value = tmp_node.g_value + s.time - tmp_node.time
            s.f_value = s.g_value + h_func(s.loc,goal)
            s.parent = tmp_node
            if s in g_score:
                pre_s = g_score[s]
                if s.g_value < pre_s.g_value:
                    pre_s.g_value = s.g_value
                    pre_s.f_value = s.f_value
                    pre_s.parent = s.parent
                    heapq.heapify(openlist)
            else:
                heapq.heappush(openlist,(s.f_value,s))
                g_score[s] = s

    path_final = []
    path_final.append(start)
    for i in range(len(path_success)-1):
        for j in range(path_success[i+1].time - path_success[i].time-1):
            path_final.append(path_success[i].loc)
        path_final.append(path_success[i+1].loc)

    return path_final


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