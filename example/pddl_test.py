import os, sys
sys.path.extend("../")

from lib_piglet.domains import pddl
from lib_piglet.expanders.pddl_expander import pddl_expander
from lib_piglet.search.tree_search import tree_search
from lib_piglet.search.graph_search import graph_search
from lib_piglet.search.search_node import compare_node_g, compare_node_f
from lib_piglet.utils.data_structure import queue,stack,bin_heap
from lib_piglet.cli.cli_tool import statistic_template, print_header
# from lib_piglet.heuristics import n_puzzle_h
from lib_piglet.search.iterative_deepening import iterative_deepening, ID_threshold

file_folder = os.path.dirname(os.path.abspath(__file__))
domainfile = os.path.join(file_folder, "pddl/pacman/pacman_bool.pddl")
problemfile = os.path.join(file_folder, "pddl/pacman/pb1.pddl")

domain = pddl.pddl(domainfile, problemfile)

print("TYPE:", domain.parser_.types)
print("OBJ:", domain.parser_.objects)

domain.parser_.add_pddl_object('f4', 'capsule')
print("OBJ :", domain.parser_.objects)
print("-"*100)
print("STATE:", domain.parser_.state)
print("adding (enemy_alive) to state...")
domain.parser_.add_to_state(['enemy alive', ])
print("STATE:", domain.parser_.state)
print("removing food_at_playground f2 from state...")
domain.parser_.remove_from_state(['food_at_playground', 'f2'])
print("STATE:", domain.parser_.state)
print("-"*100)
print("POS GOAL:", domain.parser_.positive_goals)
print("reseting positive goals to add at_home")
domain.parser_.set_positive_goals([('food_gained', 'f2'), 
                                   ('food_gained', 'f3'), 
                                   ('food_gained', 'f1'), 
                                   ('at_home', )])
print("POS GOAL:", domain.parser_.positive_goals)
print("-"*100)
print("NEG GOAL:", domain.parser_.negative_goals)
print("reseting negative goals to add not enemy_alive")
domain.parser_.set_negative_goals([('enemy_at_home', ), 
                                   ('enemy_alive', )])
print("NEG GOAL:", domain.parser_.negative_goals)
print("-"*100)

expander = pddl_expander(domain)
search = graph_search(bin_heap(compare_node_f), expander,time_limit=60)
solution = search.get_path(domain.start_state_, domain.goal_state_)

print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], "Hidden"))


