import os,sys
sys.path.extend("../")
from lib_piglet.domains import gridmap
from lib_piglet.expanders.grid_expander import grid_expander
from lib_piglet.search.tree_search import tree_search
from lib_piglet.search.graph_search import graph_search
from lib_piglet.utils.data_structure import bin_heap,stack,queue
from lib_piglet.search.search_node import compare_node_g, compare_node_f
from lib_piglet.cli.cli_tool import print_header, statistic_template
from lib_piglet.heuristics import gridmap_h
from lib_piglet.search.iterative_deepening import iterative_deepening, ID_threshold
from lib_piglet.constraints.grid_constraints import grid4_constraint,grid_constraint_table,grid_reservation_table

file_folder = os.path.dirname(os.path.abspath(__file__))
inputfile = os.path.join(file_folder, "gridmap/empty-16-16.map")



gm = gridmap.gridmap(inputfile)

expander = grid_expander(gm)

search = graph_search(bin_heap(compare_node_f), expander,heuristic_function=gridmap_h.manhattan_heuristic)
solution = search.get_path((1,2),(10,2))
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))

constraint_table = grid_constraint_table(gm.width_, gm.height_,grid4_constraint)
reservation_table = grid_reservation_table(gm.width_, gm.height_)
for i in range(1, len(solution.paths_)):
    constraint = grid4_constraint()
    constraint.v_ = True
    constraint_table.add_constraint(solution.paths_[i].state_,i,constraint)
    reservation_table.add_loc(solution.paths_[i].state_,i,1)
paths = solution.paths_





expander.constraint_table_ = constraint_table
search = graph_search(bin_heap(compare_node_f), expander,heuristic_function=gridmap_h.manhattan_heuristic)
solution = search.get_path((1,2),(10,2))
print(statistic_template.format("","",*[str(x) for x in search.get_statistic()], str(search.solution_)))


print(reservation_table.table_)
for i in range(0, len(paths)):
    print(reservation_table.is_reserved(paths[i].state_,i,-1))