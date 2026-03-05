# A* Pathfinding Algorithm

A complete, production-ready implementation of the A* (A-star) search algorithm for optimal pathfinding on 2D grids.

---

## Table of Contents

1. [What is A*?](#what-is-a)
2. [How A* Works](#how-a-works)
3. [Implementation Details](#implementation-details)
4. [Usage](#usage)
5. [Running the Demo](#running-the-demo)
6. [Requirements](#requirements)

---

## What is A*?

**A*** (pronounced "A-star") is a graph traversal and pathfinding algorithm that finds the shortest path between two points. It's widely used in:

- **Video games**: NPC navigation, RTS unit movement
- **Robotics**: Autonomous vehicle path planning
- **GPS navigation**: Finding optimal routes
- **AI**: Puzzle solving, maze traversal
- **Network routing**: Packet routing in constrained networks

### Why A*?

A* is optimal and complete:
- **Optimal**: It always finds the shortest path (given an admissible heuristic)
- **Complete**: It will always find a path if one exists
- **Efficient**: It explores fewer nodes than Dijkstra's algorithm by using a heuristic

### A* vs Dijkstra vs BFS

| Algorithm | Guarantee | Speed | Use Case |
|-----------|-----------|-------|----------|
| **BFS** | Shortest path (unweighted) | Fast | Small grids, no costs |
| **Dijkstra** | Shortest path (weighted) | Medium | Variable movement costs |
| **A*** | Shortest path | Fastest | When you have a good heuristic |

---

## How A* Works

### The Core Formula

At each step, A* evaluates nodes using:

```
f(n) = g(n) + h(n)
```

Where:
- **g(n)**: The actual cost from the start node to node `n`
- **h(n)**: The heuristic — an estimated cost from node `n` to the goal
- **f(n)**: The total estimated cost of the path through node `n`

### The Heuristic Function

The heuristic `h(n)` is what makes A* intelligent. It guides the search toward the goal.

**Properties of a good heuristic:**
- **Admissible**: Never overestimates the true cost (h(n) ≤ actual cost)
- **Consistent**: h(n) ≤ cost(n→n') + h(n')

**Common heuristics:**

| Grid Type | Heuristic | Formula |
|-----------|-----------|---------|
| 4-directional | Manhattan Distance | `\|x1-x2\| + \|y1-y2\|` |
| 8-directional | Diagonal/Chebyshev | `max(\|dx\|, \|dy\|)` |
| Any direction | Euclidean Distance | `√(dx² + dy²)` |

This implementation uses **Manhattan Distance** for 4-directional grid movement.

### Algorithm Steps

```
1. Initialize:
   - Create start node with g=0, h=heuristic(start, goal)
   - Add start node to OPEN set (priority queue sorted by f)
   - CLOSED set = empty

2. While OPEN is not empty:
   a. Pop node with lowest f score from OPEN → current
   b. If current == goal: reconstruct path and return
   c. Add current to CLOSED
   d. For each neighbor of current:
      - Skip if in CLOSED or is obstacle
      - Calculate tentative_g = current.g + movement_cost
      - If neighbor not in OPEN:
        * Create node, set g=tentative_g, calculate h and f
        * Add to OPEN
      - Else if tentative_g < neighbor.g:
        * Update neighbor.g and neighbor.f
        * Set neighbor.parent = current

3. If OPEN is empty and goal not reached:
   - No path exists
```

### Why This Works

- **g(n)** ensures we don't revisit expensive paths (like Dijkstra)
- **h(n)** focuses the search toward the goal (unlike BFS which explores equally)
- **f(n)** balances exploration vs exploitation

---

## Implementation Details

### Node Class

```python
class Node:
    def __init__(self, position, parent=None):
        self.position = position  # (x, y) tuple
        self.parent = parent      # Reference to previous node
        self.g = 0               # Cost from start
        self.h = 0               # Heuristic to goal
        self.f = 0               # Total cost (g + h)
```

Each node knows:
- Where it is (`position`)
- How it got there (`parent`) — for path reconstruction
- Its costs (`g`, `h`, `f`)

### Priority Queue with Heapq

We use Python's `heapq` for the OPEN set:

```python
import heapq

open_set = []
heapq.heappush(open_set, start_node)  # O(log n)
current = heapq.heappop(open_set)      # O(log n)
```

This gives us O(log n) insertion and extraction, much faster than a list (O(n)).

### Neighbor Generation

For 4-directional movement:

```python
directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Up, Down, Right, Left

for dx, dy in directions:
    nx, ny = x + dx, y + dy
    if is_valid(nx, ny) and not is_obstacle(nx, ny):
        neighbors.append((nx, ny))
```

### Path Reconstruction

Once we reach the goal, we backtrack using parent pointers:

```python
path = []
node = goal_node
while node is not None:
    path.append(node.position)
    node = node.parent
return path[::-1]  # Reverse to get start → goal
```

### Complexity Analysis

| Aspect | Complexity |
|--------|------------|
| Time | O(b^d) where b = branching factor, d = solution depth |
| Space | O(b^d) for storing open/closed sets |
| With good heuristic | Often much closer to O(d) |

---

## Usage

### Basic Example

```python
from astar_pathfinding import AStarPathfinder
import numpy as np

# Create a 10x10 grid (0 = free, 1 = obstacle)
grid = np.array([
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 0, 0],
])

# Initialize
finder = AStarPathfinder(grid)

# Find path
start = (0, 0)
goal = (4, 4)
path = finder.find_path(start, goal)

if path:
    print(f"Path found: {path}")
else:
    print("No path exists!")
```

### With Visualization

```python
from astar_pathfinding import AStarPathfinder, create_sample_grid

# Generate random grid
grid = create_sample_grid(size=20, obstacle_prob=0.2, seed=42)

finder = AStarPathfinder(grid)
path = finder.find_path((0, 0), (19, 19))

# Visualize (requires matplotlib)
finder.visualize(path, start=(0, 0), goal=(19, 19))
```

### Custom Grid

```python
import numpy as np

# Create your own maze
maze = np.zeros((15, 15), dtype=int)

# Add walls
maze[5:10, 7] = 1  # Vertical wall
maze[7, 3:12] = 1  # Horizontal wall

# Add gaps to make it solvable
maze[7, 5] = 0
maze[7, 10] = 0

finder = AStarPathfinder(maze)
path = finder.find_path((0, 0), (14, 14))
```

---

## Running the Demo

```bash
python astar_pathfinding.py
```

This runs two test cases:

1. **Random Grid Test**: 20×20 grid with 25% obstacles
   - Finds path from (0,0) to (19,19)
   - Displays grid, path, start (green), goal (red)

2. **Maze Test**: Structured maze with corridors
   - Tests algorithm's ability to navigate complex obstacles

---

## Requirements

- **Python**: 3.7+
- **numpy**: For grid representation
- **matplotlib**: Optional, for visualization

Install dependencies:

```bash
pip install numpy matplotlib
```

Or run without visualization (numpy only):

```bash
pip install numpy
```

---

## Extending the Implementation

### Adding Diagonal Movement

Change `get_neighbors()` to include diagonals:

```python
directions = [
    (0, 1), (0, -1), (1, 0), (-1, 0),    # Cardinal
    (1, 1), (1, -1), (-1, 1), (-1, -1)   # Diagonal
]
```

And update the heuristic to Diagonal/Chebyshev distance.

### Weighted Grids

Modify movement cost based on terrain:

```python
terrain_costs = {
    0: 1,   # Grass
    2: 3,   # Mud
    3: 5,   # Water
}
```

### Different Heuristics

```python
def euclidean_heuristic(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
```

---

## License

MIT License — Feel free to use in your projects!

---

## References

- [A* Search Algorithm - Wikipedia](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [Red Blob Games: Introduction to A*](https://www.redblobgames.com/pathfinding/a-star/introduction.html)
- [Amit's A* Pages](http://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html)
