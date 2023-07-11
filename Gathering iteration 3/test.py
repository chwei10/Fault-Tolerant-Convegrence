import networkx as nx
import matplotlib.pyplot as plt
from orginal_my_robot import Robot

def is_visibility_graph_connected(robots):
    # Create a set of visited robots
    visited = set()

    # Perform a depth-first search (DFS) starting from the first robot
    dfs(robots[0], visited)

    # Check if all robots have been visited
    return len(visited) == len(robots)

def dfs(robot, visited):
    visited.add(robot)

    # Perform DFS on neighboring robots within the observation radius
    for other_robot in robot.observe(robots):
        if other_robot not in visited:
            dfs(other_robot, visited)


# Create a list of robots with their respective parameters
robots = [
    Robot(id=0, observation_radius=50, speed=10, x=200, y=200),
    Robot(id=1, observation_radius=50, speed=10, x=30, y=30),
    Robot(id=2, observation_radius=50, speed=10, x=50, y=50),
    Robot(id=3, observation_radius=50, speed=10, x=70, y=70),
]

# Call the `is_visibility_graph_connected()` function to check connectivity
connected = is_visibility_graph_connected(robots)

print("Visibility Graph Connected:", connected)

