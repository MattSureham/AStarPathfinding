# A* Pathfinding Algorithm

A clean, well-documented implementation of the A* (A-star) search algorithm for pathfinding on 2D grids.

## What is A*?

A* is a popular pathfinding and graph traversal algorithm known for its performance and accuracy. It finds the shortest path between two points by combining:
- **g(n)**: Actual cost from start to current node
- **h(n)**: Heuristic estimate from current node to goal
- **f(n) = g(n) + h(n)**: Total estimated cost

## Features

- ✅ Clean, object-oriented implementation
- ✅ Manhattan distance heuristic
- ✅ 4-directional movement (up, down, left, right)
- ✅ Visualization with matplotlib
- ✅ Sample grids and test cases included

## Usage

```python
from astar_pathfinding import AStarPathfinder, create_sample_grid
import numpy as np

# Create a grid (0 = free space, 1 = obstacle)
grid = create_sample_grid(size=20, obstacle_prob=0.25)

# Initialize pathfinder
pathfinder = AStarPathfinder(grid)

# Find path
start = (0, 0)
goal = (19, 19)
path = pathfinder.find_path(start, goal)

# Visualize
pathfinder.visualize(path, start, goal)
```

## Running the Demo

```bash
python astar_pathfinding.py
```

This will:
1. Generate a random 20x20 grid with obstacles
2. Find the shortest path from (0,0) to (19,19)
3. Display the result with matplotlib
4. Run a second test on a simple maze

## Algorithm Overview

1. Initialize open set with start node
2. While open set is not empty:
   - Get node with lowest f score
   - If current node is goal, reconstruct and return path
   - Move current node to closed set
   - For each neighbor:
     - Skip if already evaluated
     - Calculate tentative g score
     - If new path is better, update node and add to open set
3. If open set is empty, no path exists

## Requirements

- Python 3.7+
- numpy
- matplotlib

## License

MIT
