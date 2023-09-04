# Welcome to piglet

## Requirement

python >= 3.6

## Install

1. Clone the repo to your machine
2. Run:

```
$ python setup.py install
```

## Usage

### Commandline Interface

```
$ piglet.py --help
```

run a scenario:
```
$ python3 piglet.py -p ./example/example_n_puzzle_scenario.scen -f graph -s uniform  
```

### Piglet Library
piglet provides a variety of flexible search algorithms. These algorithms are 
able to help you to build your application.

#### Example 

To use an algorithm you need a domain instance, an expander instance and a search instance. 
```python
import os,sys
from lib_piglet.domains import gridmap
from lib_piglet.expanders.grid_expander import grid_expander
from lib_piglet.search.tree_search import tree_search
from lib_piglet.utils.data_structure import bin_heap,stack,queue

mapfile = "./example/gridmap/empty-16-16.map"

# create an instance of gridmap domain
gm = gridmap.gridmap(mapfile)

# create an instance of grid_expander and pass the girdmap instance to the expander.
expander = grid_expander(gm)

# create an instance of tree_search, and pass an open list (we use a binary heap here)
# and the expander to it.
search = tree_search(bin_heap(), expander)

# start search by proving a start state and goal state. For gridmap a state is a (x,y) tuple 
solution = search.get_path((1,2),(10,2))

# print solution
print(solution)

```
