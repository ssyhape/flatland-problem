## Introduction
This project is based on the solution given in Assignment_1 in Monash University course 5222.

## Solution
The detailed algorithm can be found in report.pdf. The following are the solutions to the problems that need to be solved and the methods used in the three cases.

### Q1

In this case, the problem that needs to be solved is the path planning problem for a single agent, because there is no need to consider the possibility of collision, so it is only necessary to use a simple A-star algorithm.

### Q2

In this case, it is necessary to plan the path of multiple agents, and the way of consideration in this case is sequential, that is, the arrangement of the paths of the agents is one after another, which means that subsequent agents can Develop strategies based on previous strategies. In this part, I used the space-time A-star algorithm and the SIPP algorithm. The specific algorithm introduction can be found in report.pdf.

### Q3

In this part, it is necessary to give an overall path planning solution for multi-agent, so here I consider using an improved version of GCBS based on CBS (collision detection search) to solve this problem, which can effectively solve problems of a certain scale. All solved. Due to time issues, the GCBS here is not an excellent solution to this problem. For subsequent improvement suggestions, I personally recommend using ECBS. Here is a search method that modifies the upper-level planning solution to focal A*. The specific implementation can be found in the paper.
