# A-and-Dijkstra-algorithm
Python implementation of A* and Dijkstra algorithm:

Implemented Dijkstra and A* algorithm to find a path between start and end point on a given map for a Point Robot (radius = 0; clearance = 0) as well as a Rigid Robot (radius and clearance can be user defined)

For the Rigid Robot case, I used Minkowski Sum method to expand the given map.

Considered the workspace as a 8 connected space, that means now you can move the robot in up, down, left, right & diagonally between up-left, up-right, down-left and down-right directions, with linear movement cost as 1 unit and diagonal movement cost as √2.

I used Half-planes and semi-algebraic models to represent the obstacle space.Illustrated optimal path generation as well as node exploration animation between start and goal point using a OpenCV interface.
