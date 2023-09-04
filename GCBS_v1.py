import copy
import heapq
from lib_piglet.utils.tools import eprint
from typing import List, Tuple
import glob, os, sys,time,json

#import necessary modules that this python scripts need.
try:
    from flatland.core.transition_map import GridTransitionMap
    from flatland.envs.agent_utils import EnvAgent
    from flatland.utils.controller import get_action, Train_Actions, Directions, check_conflict, path_controller, evaluator, remote_evaluator
except Exception as e:
    eprint("Cannot load flatland modules!")
    eprint(e)
    exit(1)


debug = True
visualizer = False

# If you want to test on specific instance, turn test_single_instance to True and specify the level and test number
test_single_instance = True
level = 1
test = 7


class CBS_node:
    def __init__(self):
        self.edgeconstraints = None
        self.vertexconstraints = None
        self.solutions = None
        self.hc = None

    def __lt__(self, other):
        return self.hc < other.hc
class A_star_node:
    def __init__(self,position,direction,t):
        self.position = position
        self.direction = direction
        self.t = t
        self.f = 0
        self.h = 0
        self.g = 0
        self.parent = None

    def __lt__(self, other):
        return self.h < other.h

    def __hash__(self):
        return hash((self.position[0],self.position[1],self.direction,self.t))

    def __eq__(self, other):
        if not isinstance(other,A_star_node):
            return False
        return self.position == other.position and self.direction == other.direction and self.t == other.t
def h_func(loc,goal):
    return abs(loc[0]-goal[0])+abs(loc[1]-goal[1])


def conflict_pair(now_path,existing_paths):
    #Compute conflict pairs based on before paths
    conflict_num = 0

    if len(now_path) == 1:
        return 0

    for p in existing_paths:
        for t in range(1,min(len(now_path),len(p))):
            if p[t] == now_path[t] or (p[t] == now_path[t-1] and p[t-1] == now_path[t]):
                conflict_num +=1
                break

    return conflict_num


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

def getsuccessors(current,rail,existing_paths,vertex_constraints,edge_constraints,goal,agent_id):
    successors = []
    valid_transitions = rail.get_transitions(current.position[0], current.position[1], current.direction)
    loc_pre = current.position
    t_now = current.t + 1
    stay_node = A_star_node(loc_pre,current.direction,t_now)
    stay_node.h = h_func(loc_pre,goal)
    if conflict_k1(t_now,existing_paths,loc_pre,loc_pre) or conflict_k2(t_now,existing_paths,loc_pre,loc_pre) or (agent_id,t_now,loc_pre) in vertex_constraints:
        pass
    else:
        successors.append(stay_node)

    for i in range(len(valid_transitions)):
        loc_curr = loc_compute(loc_pre,i)
        move_node = A_star_node(loc_curr,i,t_now)
        move_node.h = h_func(loc_curr,goal)
        if valid_transitions[i]==1:

            if conflict_k1(t_now, existing_paths, loc_pre, loc_curr) or conflict_k2(t_now, existing_paths, loc_pre,
                                                                                   loc_curr) or (agent_id,t_now,loc_curr) in vertex_constraints or (agent_id,t_now,loc_pre,loc_curr) in edge_constraints:
                pass
            else :
                successors.append(move_node)

    return successors


def A_star_normal_hc(start: tuple,start_direction: int,goal: tuple,rail: GridTransitionMap,existing_paths: list, vertex_constraints: list,edge_constraints:list,agent_id , start_stay:int):
    openlist = []
    g_score = {}
    closedic = {}

    #Initialization
    start_node = A_star_node(start,start_direction,0)
    start_node.h = h_func(start,goal)
    start_node.f = start_node.g + start_node.h

    if start_stay > 0:

        tmp = A_star_node(start,start_direction,1)
        tmp.h = h_func(start,goal)
        tmp.f = start_node.g + start_node.h
        tmp.parent = start_node
        start_node = tmp

    #PUT IN
    heapq.heappush(openlist,(start_node.f,start_node))
    g_score[start_node] = start_node

    while g_score:
        _,current =  heapq.heappop(openlist)
        del g_score[current]
        closedic[current] = current

        if current.position == goal:
            path = []
            while current:
                path.append(current.position)
                current = current.parent
            path = path[::-1]
            return path

        for s in getsuccessors(current,rail,existing_paths,vertex_constraints,edge_constraints,goal,agent_id):
            if s in closedic:
                continue
            s.g = current.g + 1
            s.f = s.g + s.h
            s.parent = current
            if s in g_score:
                pre_s = g_score[s]

                if s.g < pre_s.g:
                    pre_s.g = s.g
                    pre_s.f = s.f
                    pre_s.parent =s.parent
                    heapq.heapify(openlist)
            else:
                heapq.heappush(openlist,(s.f,s))
                g_score[s] = s

    return []


def get_solution_hc(path_all):
    """
    Return: The number of pairs of conflict paths
    """
    conflict_pair_num = 0
    for i in range(len(path_all)-1):
        tmp_pair = conflict_pair(path_all[i],path_all[i+1:])
        conflict_pair_num+=tmp_pair

    return conflict_pair_num

def get_new_constraints(path_all):
    new_vertex_constraints = [[] for _ in range(len(path_all))]
    new_edge_constraints = [[] for _ in range(len(path_all))]
    for i in range(len(path_all)-1):
        for j in range(i+1,len(path_all)):
            for t in range(0,min(len(path_all[i]),len(path_all[j]))):
                if  path_all[i][t] == path_all[j][t]:
                    new_vertex_constraints[i].append((i,t,path_all[i][t]))
                    new_vertex_constraints[j].append((j,t,path_all[j][t]))
                if path_all[i][t] == path_all[j][t-1] and path_all[i][t-1] == path_all[j][t]:
                    new_edge_constraints[i].append((i,t,path_all[i][t-1],path_all[i][t]))
                    new_edge_constraints[j].append((j,t,path_all[j][t-1],path_all[j][t]))
    return new_vertex_constraints,new_edge_constraints

def get_conflict_id(new_vertex_constraints,new_edge_constraints):
    id_set = []
    for id in range(len(new_vertex_constraints)):
        if any(new_vertex_constraints[id]):
            id_set.append(id)
    for id in range(len(new_edge_constraints)):
        if any(new_edge_constraints[id]):
            id_set.append(id)

    return list(set(id_set))


def get_path(agents: List[EnvAgent],rail: GridTransitionMap, max_timestep: int):
    root = CBS_node()
    root.vertexconstraints = [[] for _ in range(len(agents))]
    root.edgeconstraints = [[] for _ in range(len(agents))]
    root.solutions = []
    for agent_id in range(0,len(agents)):
        loc = agents[agent_id].initial_position
        direction = agents[agent_id].initial_direction
        goal = agents[agent_id].target
        path = A_star_normal_hc(loc,direction,goal,rail,root.solutions,root.vertexconstraints,root.edgeconstraints,agent_id,1)
        root.solutions.append(path)

    for i, path in enumerate(root.solutions):
        if path == []:
            loc = agents[i].initial_position
            direction = agents[i].initial_direction
            goal = agents[i].target
            path = A_star_normal_hc(loc, direction, goal, rail, [], root.vertexconstraints[i],
                                    root.edgeconstraints[i],
                                    i,1)
            root.solutions[i] = path
    root.hc = get_solution_hc(root.solutions)

    openlist = []
    heapq.heappush(openlist,(root.hc,root))

    while openlist:
        _,P = heapq.heappop(openlist)
        new_vertex_constraints,new_edge_constraints = get_new_constraints(P.solutions)

        if not (any(new_vertex_constraints) or any(new_edge_constraints)):
            return P.solutions

        agents_conflict = get_conflict_id(new_vertex_constraints,new_edge_constraints)
        for agent_id in agents_conflict:
            A = CBS_node()
            A.vertexconstraints = copy.deepcopy(P.vertexconstraints)
            A.edgeconstraints = copy.deepcopy(P.edgeconstraints)
            A.vertexconstraints[agent_id] = A.vertexconstraints[agent_id] + new_vertex_constraints[agent_id]
            A.edgeconstraints[agent_id] = A.edgeconstraints[agent_id] + new_edge_constraints[agent_id]
            A.solutions = copy.deepcopy(P.solutions)
            agents_need_modified = []
            for id in range(len(agents)):
                if any(A.vertexconstraints[id]) or any(A.edgeconstraints[id]):
                    agents_need_modified.append(id)

            for id in agents_need_modified:
                loc = agents[id].initial_position
                direction = agents[id].initial_direction
                goal = agents[id].target
                path = A_star_normal_hc(loc, direction, goal, rail, A.solutions[:id], A.vertexconstraints[id],A.edgeconstraints[id],
                                        id,1)
                A.solutions[id] = path

            #Make sure generate before is good.
            for i,path in enumerate(A.solutions):
                if path == []:
                    loc = agents[i].initial_position
                    direction = agents[i].initial_direction
                    goal = agents[i].target
                    path = A_star_normal_hc(loc, direction, goal, rail,[], A.vertexconstraints[i],
                                            A.edgeconstraints[i],
                                            i,1)
                    A.solutions[i] =path

            A.hc = get_solution_hc(A.solutions)

            heapq.heappush(openlist,(A.hc,A))

    return []

def get_replan_paths(agents: List[EnvAgent],rail: GridTransitionMap, max_timestep: int):
    root = CBS_node()
    root.vertexconstraints = [[] for _ in range(len(agents))]
    root.edgeconstraints = [[] for _ in range(len(agents))]
    root.solutions = []
    for agent_id in range(0,len(agents)):
        loc = agents[agent_id].old_position
        direction = agents[agent_id].old_direction
        goal = agents[agent_id].target
        path = A_star_normal_hc(loc,direction,goal,rail,root.solutions,root.vertexconstraints,root.edgeconstraints,agent_id,agents[agent_id].malfunction_data["malfunction"])
        root.solutions.append(path)

    for i, path in enumerate(root.solutions):
        if path == []:
            loc = agents[i].old_position
            direction = agents[i].old_direction
            goal = agents[i].target
            path = A_star_normal_hc(loc, direction, goal, rail, [], root.vertexconstraints[i],
                                    root.edgeconstraints[i],
                                    i,agents[i].malfunction_data["malfunction"])
            root.solutions[i] = path
    root.hc = get_solution_hc(root.solutions)

    openlist = []
    heapq.heappush(openlist,(root.hc,root))

    while openlist:
        _,P = heapq.heappop(openlist)
        new_vertex_constraints,new_edge_constraints = get_new_constraints(P.solutions)

        if any(not sublist for sublist in P.solutions):
            continue
        if not (any(new_vertex_constraints) or any(new_edge_constraints)):
            return P.solutions

        agents_conflict = get_conflict_id(new_vertex_constraints,new_edge_constraints)
        for agent_id in agents_conflict:
            A = CBS_node()
            A.vertexconstraints = copy.deepcopy(P.vertexconstraints)
            A.edgeconstraints = copy.deepcopy(P.edgeconstraints)
            A.vertexconstraints[agent_id] = A.vertexconstraints[agent_id] + new_vertex_constraints[agent_id]
            A.edgeconstraints[agent_id] = A.edgeconstraints[agent_id] + new_edge_constraints[agent_id]
            A.solutions = copy.deepcopy(P.solutions)

            agents_need_modified = []
            for id in range(len(agents)):
                if any(A.vertexconstraints[id]) or any(A.edgeconstraints[id]):
                    agents_need_modified.append(id)

            for id in agents_need_modified:
                loc = agents[id].old_position
                direction = agents[id].old_direction
                goal = agents[id].target
                path = A_star_normal_hc(loc, direction, goal, rail, A.solutions[:id]+A.solutions[id+1:], A.vertexconstraints[id],A.edgeconstraints[id],
                                        id,agents[id].malfunction_data["malfunction"])
                A.solutions[id] = path

            """
            loc = agents[agent_id].position
            direction = agents[agent_id].direction
            goal = agents[agent_id].target
            path = A_star_normal_hc(loc, direction, goal, rail, [], A.vertexconstraints[agent_id],A.edgeconstraints[agent_id],
                                        agent_id,agents[agent_id].malfunction_data["malfunction"])
            A.solutions[agent_id] = path
            """
            #Make sure generate before is good.
            for i,path in enumerate(A.solutions):
                if path == []:
                    loc = agents[i].old_position
                    direction = agents[i].old_direction
                    goal = agents[i].target
                    path = A_star_normal_hc(loc, direction, goal, rail,[], A.vertexconstraints[i],
                                            A.edgeconstraints[i],
                                            i,agents[i].malfunction_data["malfunction"])
                    A.solutions[i] =path

            A.hc = get_solution_hc(A.solutions)

            heapq.heappush(openlist,(A.hc,A))

    return []


def replan(agents: List[EnvAgent],rail: GridTransitionMap,  current_timestep: int, existing_paths: List[Tuple], max_timestep:int, new_malfunction_agents: List[int], failed_agents: List[int]):
    if debug:
        print("Replan function not implemented yet!",file=sys.stderr)

    #There is  a point is to update path which begins from current_timestep

    #TODO : path_before_current means the paths before current timme
    # get the paths not finished
    paths_before_current_time = [sublist[:min(len(sublist),current_timestep)] for sublist in existing_paths]
    unfinished_id = []
    for agent_id in range(len(agents)):
        if existing_paths[agent_id][min(len(existing_paths[agent_id])-1,current_timestep)] != agents[agent_id].target or (existing_paths[agent_id][min(len(existing_paths[agent_id])-1,current_timestep)] == agents[agent_id].target and agents[agent_id].malfunction_data["malfunction"]>0 and agents[agent_id].position!=None) :
            unfinished_id.append(agent_id)

    agents_re = [agents[id] for id in unfinished_id]
    path_after_timespace = get_replan_paths(agents_re,rail,max_timestep)

    for i,id in enumerate(unfinished_id):
        existing_paths[id] = paths_before_current_time[id][:-1] + path_after_timespace[i]

    """
    # new start
    after_path_start = existing_paths[:][current_timestep]

    # new start of new_malfunction
    for agent_id in new_malfunction_agents:
        after_path_start[agent_id] = paths_befor_current[agent_id][current_timestep-1]

    path_for_print = []
    success_find= False
    root = CBS_node()
    # Only considering the situation of vertex_constraints
    root.vertexconstraints = [[] for _ in range(len(agents))]
    root.edgeconstraints = [[] for _ in range(len(agents))]
    #update path before
    for agent_id in new_malfunction_agents:
        stay_position = existing_paths[agent_id][current_timestep-1]
        for time_of_stay in range(agents[agent_id].malfunction_data["malfunction"]):
            existing_paths[agent_id].insert(current_timestep,stay_position)

    root.solutions = existing_paths
    #Update the path
    for agent_id in range(len(agents)):
        if agent_id not in new_malfunction_agents:
            loc = agents[agent_id].initial_position
            direction = agents[agent_id].initial_direction
            goal = agents[agent_id].target
            path = A_star_normal_hc(loc, direction, goal, rail, root.solutions[:agent_id] + root.solutions[agent_id + 1:],
                                root.vertexconstraints[agent_id], root.edgeconstraints[agent_id],agent_id)
            root.solutions[agent_id] = path

    root.hc = get_solution_hc(root.solutions)

    openlist = []
    heapq.heappush(openlist,(root.hc,root))

    while openlist:
        _,P = heapq.heappop(openlist)
        new_vertex_constraints,new_edge_constraints = get_new_constraints(P.solutions)

        if not (any(new_vertex_constraints) or any(new_edge_constraints)):
            path_for_print = P.solutions
            success_find = True
            break

        agents_conflict = get_conflict_id(new_vertex_constraints,new_edge_constraints)
        for agent_id in agents_conflict and agent_id not in new_malfunction_agents:
            A = CBS_node()
            A.vertexconstraints = copy.deepcopy(P.vertexconstraints)
            A.edgeconstraints = copy.deepcopy(P.vertexconstraints)
            A.vertexconstraints[agent_id] = A.vertexconstraints[agent_id] + new_vertex_constraints[agent_id]
            A.edgeconstraints[agent_id] = A.edgeconstraints[agent_id] + new_edge_constraints[agent_id]
            A.solutions = copy.deepcopy(P.solutions)
            agents_need_modified = []
            for id in range(len(agents)):
                if any(A.vertexconstraints[id]) or any(A.edgeconstraints[id]):
                    agents_need_modified.append(id)

            for id in agents_need_modified:
                loc = agents[id].initial_position
                direction = agents[id].initial_direction
                goal = agents[id].target
                path = A_star_normal_hc(loc, direction, goal, rail, A.solutions[:id]+A.solutions[id+1:], A.vertexconstraints[id],A.edgeconstraints[id],
                                        id)
                A.solutions[id] = path

            #Make sure generate before is good.
            for i,path in enumerate(A.solutions):
                if path == []:
                    loc = agents[i].initial_position
                    direction = agents[i].initial_direction
                    goal = agents[i].target
                    path = A_star_normal_hc(loc, direction, goal, rail,[], A.vertexconstraints[i],
                                            A.edgeconstraints[i],
                                            i)
                    A.solutions[i] = path

            A.hc = get_solution_hc(A.solutions)

            heapq.heappush(openlist,(A.hc,A))
    if success_find and debug:
        for id in range(len(path_for_print)):
            print("agent ",id,"is :",path_for_print[id])

        return  path_for_print

    """
    return existing_paths
if __name__ == "__main__":

    if len(sys.argv) > 1:
        remote_evaluator(get_path,sys.argv, replan = replan)
    else:
        script_path = os.path.dirname(os.path.abspath(__file__))
        test_cases = glob.glob(os.path.join(script_path, "multi_test_case/level*_test_*.pkl"))

        if test_single_instance:
            test_cases = glob.glob(os.path.join(script_path,"multi_test_case/level{}_test_{}.pkl".format(level, test)))
        test_cases.sort()
        deadline_files =  [test.replace(".pkl",".ddl") for test in test_cases]
        evaluator(get_path, test_cases, debug, visualizer, 3, deadline_files, replan = replan)




