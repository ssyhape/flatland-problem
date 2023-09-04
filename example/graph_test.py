import os,sys
sys.path.extend("../")
from lib_piglet.domains import graph
from lib_piglet.expanders.graph_expander import graph_expander
from lib_piglet.search.tree_search import tree_search
from lib_piglet.search.graph_search import graph_search
from lib_piglet.utils.data_structure import bin_heap,stack,queue
from lib_piglet.search.search_node import compare_node_g, compare_node_f
from lib_piglet.cli.cli_tool import print_header, statistic_template
from lib_piglet.heuristics import graph_h
from lib_piglet.search.iterative_deepening import iterative_deepening, ID_threshold

file_folder = os.path.dirname(os.path.abspath(__file__))
inputfile = os.path.join(file_folder, "graphmap/sample.graph")



gm = graph.graph(inputfile)

expander = graph_expander(gm)
print_header()



search = tree_search(stack(), expander,time_limit=10)
path = search.get_path(gm.get_vertex(1),gm.get_vertex(5))
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))
search = tree_search(queue(), expander,time_limit=10)
path = search.get_path(gm.get_vertex(1),gm.get_vertex(5))
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))
search = graph_search(queue(), expander,time_limit=10)
path = search.get_path(gm.get_vertex(1),gm.get_vertex(5))
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))
search = graph_search(stack(), expander,time_limit=10)
path = search.get_path(gm.get_vertex(1),gm.get_vertex(5))
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))
search = graph_search(bin_heap(compare_node_g), expander,time_limit=10)
path = search.get_path(gm.get_vertex(1),gm.get_vertex(5))
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))

search = graph_search(bin_heap(compare_node_f), expander,heuristic_function=graph_h.straight_heuristic,time_limit=10)
path = search.get_path(gm.get_vertex(1),gm.get_vertex(5))
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))


# search = iterative_deepening(stack(), expander,heuristic_function=graph_h.straight_heuristic,time_limit=10)
# path = search.get_path(gm.get_vertex(1),gm.get_vertex(5),threshold_type=ID_threshold.cost)
# print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))

search = iterative_deepening(stack(), expander,heuristic_function=None,time_limit=10)
path = search.get_path(gm.get_vertex(1),gm.get_vertex(5),threshold_type=ID_threshold.depth)
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))










