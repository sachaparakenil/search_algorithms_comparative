# -*- coding: utf-8 -*-
"""search_algorithms_comparative.ipynb
"""

import random
from heapq import heappop, heappush
from collections import deque

# Depth-First Search (DFS)
def dfs(graph, start, goal):
    stack = [(start, [start])]
    visited = set()

    while stack:
        current, path = stack.pop()
        if current == goal:
            return path
        if current not in visited:
            visited.add(current)
            stack.extend((neighbor, path + [neighbor]) for neighbor in graph[current] - visited)
    return None

# Breadth-First Search (BFS)
def bfs(graph, start, goal):
    queue = deque([(start, [start])])
    visited = set()

    while queue:
        current, path = queue.popleft()
        if current == goal:
            return path
        if current not in visited:
            visited.add(current)
            queue.extend((neighbor, path + [neighbor]) for neighbor in graph[current] - visited)
    return None

# Random-Walk
def random_walk(graph, start, goal, max_steps=1000):
    current = start
    path = [start]
    visited = set()
    steps = 0

    while current != goal and steps < max_steps:
        visited.add(current)
        neighbors = list(graph[current] - visited)
        if not neighbors:
            return None
        current = random.choice(neighbors)
        path.append(current)
        steps += 1
    return path if current == goal else None

# Hill-Climbing Search
def hill_climbing(graph, start, goal, heuristic):
    current = start
    path = [start]
    visited = set()

    while current != goal:
        visited.add(current)
        neighbors = list(graph[current] - visited)
        if not neighbors:
            return None
        current = min(neighbors, key=heuristic)
        path.append(current)
    return path

# Best-First Search
def best_first_search(graph, start, goal, heuristic):
    open_list = [(0, start, [start])]
    visited = set()

    while open_list:
        _, current, path = heappop(open_list)
        if current == goal:
            return path
        if current not in visited:
            visited.add(current)
            for neighbor in graph[current] - visited:
                heappush(open_list, (heuristic(neighbor), neighbor, path + [neighbor]))
    return None

# A* Search
def a_star_search(graph, start, goal, heuristic):
    open_list = [(0, start, [start])]
    g_costs = {start: 0}
    visited = set()

    while open_list:
        _, current, path = heappop(open_list)
        if current == goal:
            return path
        if current not in visited:
            visited.add(current)
            for neighbor in graph[current] - visited:
                tentative_g_cost = g_costs[current] + 1  # Assuming all edges have a cost of 1
                if neighbor not in g_costs or tentative_g_cost < g_costs[neighbor]:
                    g_costs[neighbor] = tentative_g_cost
                    f_cost = tentative_g_cost + heuristic(neighbor)
                    heappush(open_list, (f_cost, neighbor, path + [neighbor]))
    return None

# Custom Search (Improved Dijkstra’s Algorithm)
def custom_dijkstra(graph, start, goal):
    open_list = [(0, start, [start])]
    distances = {start: 0}
    visited = set()

    while open_list:
        current_distance, current, path = heappop(open_list)
        if current == goal:
            return path
        if current not in visited:
            visited.add(current)
            for neighbor in graph[current] - visited:
                distance = current_distance + 1  # Assuming all edges have a cost of 1
                if neighbor not in distances or distance < distances[neighbor]:
                    distances[neighbor] = distance
                    heappush(open_list, (distance, neighbor, path + [neighbor]))
    return None

# Example Graph
graph = {
    'A': {'B', 'C'},
    'B': {'A', 'D', 'E'},
    'C': {'A', 'F'},
    'D': {'B'},
    'E': {'B', 'F'},
    'F': {'C', 'E'}
}

# Heuristic function for informed searches (example heuristic)
heuristic = lambda x: abs(ord('F') - ord(x))

# Function to run all search algorithms and print their results
def run_search_algorithms(graph, start, goal):
    algorithms = {
        "DFS": dfs,
        "BFS": bfs,
        "Random-Walk": lambda g, s, go: random_walk(g, s, go),
        "Hill-Climbing": lambda g, s, go: hill_climbing(g, s, go, heuristic),
        "Best-First Search": lambda g, s, go: best_first_search(g, s, go, heuristic),
        "A* Search": lambda g, s, go: a_star_search(g, s, go, heuristic),
        "Custom Dijkstra": custom_dijkstra
    }

    for name, func in algorithms.items():
        path = func(graph, start, goal)
        if path:
            print(f"{name}: Path found - {path}")
        else:
            print(f"{name}: No path found")

# Run and print the results of different search algorithms
if __name__ == "__main__":
    start_node = 'A'
    goal_node = 'F'
    run_search_algorithms(graph, start_node, goal_node)
