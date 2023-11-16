import math
import random
from collections import deque
from queue import Queue, PriorityQueue
from typing import Dict, List, Tuple
from numpy import inf


#Searches for path in grid using Dijkstra logic
def Dijkstra(graph: Dict, start: Tuple, end: Tuple) -> Tuple[List, List]:

    tracker = PriorityQueue()
    visited = []

    dist = {key: inf for key in graph.keys()}
    parent = {key: None for key in graph.keys()}  # Helps backtrace the traveled path
    level = {key: -1 for key in graph.keys()}

    dist[start] = 0
    level[start] = 0
    tracker.put((0, start))

    iterations = 0
    found = False
    while not tracker.empty():
        iterations += 1
        current_node = tracker.get()[1]
        visited.append(current_node)

        if current_node == end:
            found = True
            break

        children = graph[current_node]
        for child_node in children:
            if child_node in visited:
                continue

            if child_node == start:
                continue

            alt = dist[current_node] + 1
            if alt < dist[child_node]:  # Relaxation
                dist[child_node] = alt
                parent[child_node] = current_node
                tracker.put((dist[child_node], child_node))    
                level[child_node] = level[current_node] + 1

    path = []
    current = end
    while current != start:
        if current not in parent:
            final_path = [None]
            break
        next = parent[current]
        path.append(next)
        current = next
    path.pop()

    return level, path, iterations, found

#Searches for path in grid by moving to a random neighboring cell at each iteration
def RandomPlanner(graph: Dict, start: Tuple, end: Tuple) -> List:

    traveled_path = []
    visited = {key: False for key in graph.keys()}
    parent = {key: None for key in graph.keys()}

    current_node = start
    visited[current_node] = True
    traveled_path.append(current_node)

    index, max_iterations = 0, 100000
    found = False
    while current_node != end:
        children = find_unvisited_children(current_node, graph, visited)
        if not children:  # If there's nowhere to go, take a step back
            if current_node not in parent:
                break
            current_node = parent[current_node]
            continue

        child = random.choice(graph[current_node])
        if visited[child]:
            continue

        if child == end:  # If goal reached, terminate traversal
            found = True
            visited[child] = True
            traveled_path.append(child)
            parent[child] = current_node
            break

        visited[child] = True
        traveled_path.append(child)
        parent[child] = current_node
        current_node = child

        index += 1
        if index > max_iterations:
            break

    return traveled_path, index, found

#Finds the unvisited children of a particular node
def find_unvisited_children(node: Tuple, graph: Dict, visited: Dict) -> List:
    #print(list(visited.keys()))
    unvisited_children = []
    for child_node in graph.get(node, []):
        if visited[child_node]:
            continue
        unvisited_children.append(child_node)
    return unvisited_children
