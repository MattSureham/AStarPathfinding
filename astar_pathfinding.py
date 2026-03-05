"""
A* Pathfinding Algorithm Implementation

A clean implementation of the A* (A-star) search algorithm for pathfinding
on a 2D grid with obstacles.
"""

import heapq
import numpy as np
from typing import List, Tuple, Set, Optional

# Optional matplotlib import
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


class Node:
    """Represents a node in the search graph."""
    
    def __init__(self, position: Tuple[int, int], parent: Optional['Node'] = None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost from start to current node
        self.h = 0  # Heuristic cost to goal
        self.f = 0  # Total cost (g + h)
    
    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f
    
    def __hash__(self):
        return hash(self.position)
    
    def __repr__(self):
        return f"Node({self.position}, f={self.f})"


class AStarPathfinder:
    """
    A* Pathfinding Algorithm implementation.
    
    Finds the shortest path between two points on a grid using
    the A* search algorithm with Manhattan distance heuristic.
    """
    
    def __init__(self, grid: np.ndarray):
        """
        Initialize with a grid.
        
        Args:
            grid: 2D numpy array where 0 = free space, 1 = obstacle
        """
        self.grid = grid
        self.height, self.width = grid.shape
    
    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> int:
        """
        Calculate Manhattan distance heuristic.
        
        Args:
            a: Position (x, y)
            b: Position (x, y)
        
        Returns:
            Manhattan distance between a and b
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def get_neighbors(self, position: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Get valid neighbor positions (up, down, left, right).
        
        Args:
            position: Current position (x, y)
        
        Returns:
            List of valid neighbor positions
        """
        x, y = position
        neighbors = []
        
        # 4-directional movement (up, down, left, right)
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            
            # Check bounds
            if 0 <= nx < self.width and 0 <= ny < self.height:
                # Check if not an obstacle
                if self.grid[ny, nx] == 0:
                    neighbors.append((nx, ny))
        
        return neighbors
    
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        Find path from start to goal using A* algorithm.
        
        Args:
            start: Starting position (x, y)
            goal: Goal position (x, y)
        
        Returns:
            List of positions forming the path, or None if no path exists
        """
        # Validate inputs
        if not (0 <= start[0] < self.width and 0 <= start[1] < self.height):
            raise ValueError("Start position out of bounds")
        if not (0 <= goal[0] < self.width and 0 <= goal[1] < self.height):
            raise ValueError("Goal position out of bounds")
        if self.grid[start[1], start[0]] == 1:
            raise ValueError("Start position is an obstacle")
        if self.grid[goal[1], goal[0]] == 1:
            raise ValueError("Goal position is an obstacle")
        
        # Initialize open and closed sets
        open_set: List[Node] = []
        closed_set: Set[Tuple[int, int]] = set()
        
        # Create start node
        start_node = Node(start)
        start_node.g = 0
        start_node.h = self.heuristic(start, goal)
        start_node.f = start_node.g + start_node.h
        
        # Add start node to open set
        heapq.heappush(open_set, start_node)
        
        while open_set:
            # Get node with lowest f score
            current_node = heapq.heappop(open_set)
            
            # Check if we reached the goal
            if current_node.position == goal:
                # Reconstruct path
                path = []
                node = current_node
                while node is not None:
                    path.append(node.position)
                    node = node.parent
                return path[::-1]  # Reverse to get start -> goal
            
            # Add to closed set
            closed_set.add(current_node.position)
            
            # Check all neighbors
            for neighbor_pos in self.get_neighbors(current_node.position):
                # Skip if already evaluated
                if neighbor_pos in closed_set:
                    continue
                
                # Calculate g score
                tentative_g = current_node.g + 1
                
                # Check if neighbor is in open set
                neighbor_node = None
                for node in open_set:
                    if node.position == neighbor_pos:
                        neighbor_node = node
                        break
                
                if neighbor_node is None:
                    # Create new node
                    neighbor_node = Node(neighbor_pos, current_node)
                    neighbor_node.g = tentative_g
                    neighbor_node.h = self.heuristic(neighbor_pos, goal)
                    neighbor_node.f = neighbor_node.g + neighbor_node.h
                    heapq.heappush(open_set, neighbor_node)
                elif tentative_g < neighbor_node.g:
                    # Found a better path
                    neighbor_node.parent = current_node
                    neighbor_node.g = tentative_g
                    neighbor_node.f = neighbor_node.g + neighbor_node.h
        
        # No path found
        return None
    
    def visualize(self, path: Optional[List[Tuple[int, int]]] = None, 
                  start: Optional[Tuple[int, int]] = None,
                  goal: Optional[Tuple[int, int]] = None,
                  title: str = "A* Pathfinding"):
        """
        Visualize the grid and path using matplotlib.
        
        Args:
            path: List of positions forming the path
            start: Starting position (highlighted in green)
            goal: Goal position (highlighted in red)
            title: Plot title
        """
        if not MATPLOTLIB_AVAILABLE:
            print("matplotlib not available. Install with: pip install matplotlib")
            return
        
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Display grid (0 = white/free, 1 = black/obstacle)
        ax.imshow(self.grid, cmap='binary', interpolation='nearest')
        
        # Draw path if provided
        if path:
            path_x = [p[0] for p in path]
            path_y = [p[1] for p in path]
            ax.plot(path_x, path_y, 'b-', linewidth=3, alpha=0.7, label='Path')
            ax.plot(path_x, path_y, 'bo', markersize=8, alpha=0.5)
        
        # Mark start and goal
        if start:
            ax.plot(start[0], start[1], 'go', markersize=15, label='Start')
        if goal:
            ax.plot(goal[0], goal[1], 'ro', markersize=15, label='Goal')
        
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        
        plt.tight_layout()
        plt.show()


def create_sample_grid(size: int = 20, obstacle_prob: float = 0.3, seed: int = 42) -> np.ndarray:
    """
    Create a sample grid with random obstacles.
    
    Args:
        size: Grid size (size x size)
        obstacle_prob: Probability of a cell being an obstacle
        seed: Random seed for reproducibility
    
    Returns:
        2D numpy array representing the grid
    """
    np.random.seed(seed)
    grid = np.random.choice([0, 1], size=(size, size), p=[1-obstacle_prob, obstacle_prob])
    
    # Ensure corners are free for start/goal
    grid[0, 0] = 0
    grid[size-1, size-1] = 0
    
    return grid


def main():
    """Main demonstration of A* pathfinding."""
    
    print("=" * 50)
    print("A* Pathfinding Algorithm Demo")
    print("=" * 50)
    
    # Create sample grid
    grid = create_sample_grid(size=20, obstacle_prob=0.25, seed=42)
    
    # Initialize pathfinder
    pathfinder = AStarPathfinder(grid)
    
    # Define start and goal
    start = (0, 0)
    goal = (19, 19)
    
    print(f"\nGrid size: {grid.shape}")
    print(f"Start: {start}")
    print(f"Goal: {goal}")
    print(f"Obstacles: {np.sum(grid)} / {grid.size} cells")
    
    # Find path
    print("\nSearching for path...")
    path = pathfinder.find_path(start, goal)
    
    if path:
        print(f"✓ Path found!")
        print(f"  Length: {len(path)} steps")
        print(f"  Path: {path[:5]}...{path[-5:]}" if len(path) > 10 else f"  Path: {path}")
        
        # Visualize
        pathfinder.visualize(path, start, goal, title=f"A* Pathfinding (Length: {len(path)})")
    else:
        print("✗ No path found!")
        pathfinder.visualize(start=start, goal=goal, title="A* Pathfinding - No Path Found")
    
    # Test case 2: Simple maze
    print("\n" + "=" * 50)
    print("Test Case 2: Simple Maze")
    print("=" * 50)
    
    maze = np.array([
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 1, 0],
        [0, 1, 1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 1, 0, 0, 0],
        [0, 1, 1, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
    ])
    
    maze_finder = AStarPathfinder(maze)
    maze_start = (0, 0)
    maze_goal = (7, 7)
    
    maze_path = maze_finder.find_path(maze_start, maze_goal)
    
    if maze_path:
        print(f"✓ Path found! Length: {len(maze_path)}")
        maze_finder.visualize(maze_path, maze_start, maze_goal, 
                              title=f"Maze Solution (Length: {len(maze_path)})")
    else:
        print("✗ No path found!")


if __name__ == "__main__":
    main()
