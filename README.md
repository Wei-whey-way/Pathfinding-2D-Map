# Pathfinding-2D-Map
Algorithms include: Breadth First Search, Uniform Cost Search, A* Search

Given a start position and an end position on a 2D map, find a path from the start position to the end position. Map is supplied as an input (See map.txt files for example inputs)

Example of 2D map:
```
1 1 1 1 1 1 4 7 8 X
1 1 1 1 1 1 1 5 8 8
1 1 1 1 1 1 1 4 6 7
1 1 1 1 1 X 1 1 3 6
1 1 1 1 1 X 1 1 1 1
1 1 1 1 1 1 1 1 1 1
6 1 1 1 1 X 1 1 1 1
7 7 1 X X X 1 1 1 1
8 8 1 1 1 1 1 1 1 1
X 8 7 1 1 1 1 1 1 1
```

The character 'X' denotes an obstacle that cannot be traversed by a path, while the digits represent the elevation at the respective positions. Any two points on the path are only connected if they are vertically or horizontally (but not diagonally) adjacent.

Problem Formulation
- States: Any obstacle-free position (i, j) on the map.
- Initial States: A position (i0, j0) given by the user.
- Actions: Since we consider 4-connectedness, only four actions are available: Up, down, left and right (your program must expand each node in this order). Available actions for positions adjacent to the map boundary or obstacles are reduced accordingly.
- Transition Model: Moving left moves the current position to the left, etc.
- Goal Test: Check if the current state is the end position (i*, j*) given by the user.
- Path Cost: The cost of a path is the sum of the costs between adjacent points in the path, and the cost between adjacent points is 1 plus the difference between the elevation of the two points if we climb "uphill" or simply 1 if we stay "level" or slide "downhill".


Given a map M and a Path P{(i0, j0), (i1, j1), ... (iN, jN)}, the cost of the path is calculated as:

$$
g(P) = \sum_{k=1}^{N} c(i_{k-1}, j_{k-1}, i_k, j_k, M)
$$

Where the cost function $c(a, b, c, d, M)$ is:

$$
c(a, b, c, d, M) =
\begin{cases}
1 + M(c, d) - M(a, b), & \text{if } M(c, d) - M(a, b) > 0 \\\\
1, & \text{otherwise}
\end{cases}
$$

and M(a, b) is the elevation at the position (a, b). 
