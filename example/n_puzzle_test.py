import os, sys
sys.path.extend("../")

from lib_piglet.domains import n_puzzle
from lib_piglet.expanders.n_puzzle_expander import n_puzzle_expander
from lib_piglet.search.tree_search import tree_search
from lib_piglet.search.graph_search import graph_search
from lib_piglet.search.search_node import compare_node_g, compare_node_f
from lib_piglet.utils.data_structure import queue,stack,bin_heap
from lib_piglet.cli.cli_tool import statistic_template, print_header
from lib_piglet.heuristics import n_puzzle_h
from lib_piglet.search.iterative_deepening import iterative_deepening, ID_threshold

file_folder = os.path.dirname(os.path.abspath(__file__))
inputfile = os.path.join(file_folder, "n_puzzle/sample_9_puzzle")



puzzle = n_puzzle.n_puzzle(4)
puzzle.set_start([1,2,3,4,5,6,7,8,9,"d",11,12,13,14.0,"15",10])
expander = n_puzzle_expander(puzzle)
search = tree_search(queue(), expander, time_limit=10)
path = search.get_path(puzzle.start_state(), puzzle.goal_state())
print_header()
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))

search = tree_search(stack(), expander, time_limit=10)
path = search.get_path(puzzle.start_state(), puzzle.goal_state())
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))

search = tree_search(bin_heap(compare_node_g), expander, time_limit=10)
path = search.get_path(puzzle.start_state(), puzzle.goal_state())
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))

search = graph_search(queue(), expander, time_limit=10)
path = search.get_path(puzzle.start_state(), puzzle.goal_state())
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))

search = graph_search(stack(), expander, time_limit=10)
path = search.get_path(puzzle.start_state(), puzzle.goal_state())
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))

search = graph_search(bin_heap(compare_node_g), expander, time_limit=10)
path = search.get_path(puzzle.start_state(), puzzle.goal_state())
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))

search = graph_search(bin_heap(compare_node_f), expander, time_limit=10, heuristic_function=n_puzzle_h.sum_manhattan_heuristic)
path = search.get_path(puzzle.start_state(), puzzle.goal_state())
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))

search = graph_search(bin_heap(compare_node_f), expander, time_limit=10, heuristic_function=n_puzzle_h.sum_straight_heuristic)
path = search.get_path(puzzle.start_state(), puzzle.goal_state())
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))

search = iterative_deepening(stack(), expander, time_limit=10, heuristic_function=n_puzzle_h.sum_manhattan_heuristic)
path = search.get_path(puzzle.start_state(), puzzle.goal_state(),threshold_type=ID_threshold.cost)
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))

search = iterative_deepening(stack(), expander, time_limit=10, heuristic_function=None)
path = search.get_path(puzzle.start_state(), puzzle.goal_state(), threshold_type=ID_threshold.depth)
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))



