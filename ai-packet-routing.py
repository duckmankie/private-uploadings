import networkx as nx
import matplotlib.pyplot as plt
import heapq
from collections import deque
import time

graph_data = {
    'A': {'B': 2, 'C': 3},
    'B': {'A': 2, 'D': 4, 'E': 2},
    'C': {'A': 3, 'F': 5},
    'D': {'B': 4, 'G': 1},
    'E': {'B': 2, 'F': 3, 'G': 2},
    'F': {'C': 5, 'E': 3, 'H': 2},
    'G': {'D': 1, 'E': 2, 'I': 4},
    'H': {'F': 2, 'I': 3},
    'I': {'G': 4, 'H': 3, 'J': 2},
    'J': {'I': 2}
}


# Algorithms
def uniform_cost_search(graph, start, goal):
    frontier = []
    heapq.heappush(frontier, (0, start, [start]))
    explored = set()
    visited_costs = {start: 0}
    nodes_expanded = 0
    max_frontier_size = 1

    while frontier:
        cost, node, path = heapq.heappop(frontier)

        if node in explored:
            continue

        explored.add(node)
        nodes_expanded += 1

        if node == goal:
            return {
                'path': path,
                'cost': cost,
                'nodes_expanded': nodes_expanded,
                'max_frontier_size': max_frontier_size
            }

        for neighbor, edge_cost in graph.get(node, {}).items():
            new_cost = cost + edge_cost
            if neighbor not in explored and (neighbor not in visited_costs or new_cost < visited_costs[neighbor]):
                visited_costs[neighbor] = new_cost
                heapq.heappush(frontier, (new_cost, neighbor, path + [neighbor]))
                max_frontier_size = max(max_frontier_size, len(frontier))

    return None

def breadth_first_search(graph, start, goal):
    frontier = deque()
    frontier.append((start, [start]))
    explored = set()
    nodes_expanded = 0
    max_frontier_size = 1

    while frontier:
        node, path = frontier.popleft()
        nodes_expanded += 1

        if node == goal:
            total_cost = sum(graph[path[i]][path[i+1]] for i in range(len(path)-1))
            return {
                'path': path,
                'cost': total_cost,
                'nodes_expanded': nodes_expanded,
                'max_frontier_size': max_frontier_size
            }

        if node not in explored:
            explored.add(node)
            for neighbor in graph.get(node, {}):
                if neighbor not in explored:
                    frontier.append((neighbor, path + [neighbor]))
                    max_frontier_size = max(max_frontier_size, len(frontier))

    return None

def depth_first_search(graph, start, goal):
    stack = [(start, [start])]
    explored = set()
    nodes_expanded = 0
    max_stack_size = 1

    while stack:
        node, path = stack.pop()
        nodes_expanded += 1

        if node == goal:
            total_cost = sum(graph[path[i]][path[i+1]] for i in range(len(path)-1))
            return {
                'path': path,
                'cost': total_cost,
                'nodes_expanded': nodes_expanded,
                'max_frontier_size': max_stack_size
            }

        if node not in explored:
            explored.add(node)
            for neighbor in reversed(graph.get(node, {})):
                if neighbor not in explored:
                    stack.append((neighbor, path + [neighbor]))
                    max_stack_size = max(max_stack_size, len(stack))

    return None

# Output
start_ucs = time.time()
result_ucs = uniform_cost_search(graph_data, 'A', 'J')
elapsed_ucs = time.time() - start_ucs

start_bfs = time.time()
result_bfs = breadth_first_search(graph_data, 'A', 'J')
elapsed_bfs = time.time() - start_bfs

start_dfs = time.time()
result_dfs = depth_first_search(graph_data, 'A', 'J')
elapsed_dfs = time.time() - start_dfs

for name, result, elapsed in [
    ('UCS', result_ucs, elapsed_ucs),
    ('BFS', result_bfs, elapsed_bfs),
    ('DFS', result_dfs, elapsed_dfs)
]:
    if result:
        print(f"\n{name} Result:")
        print("Path:", ' → '.join(result['path']))
        print("Cost:", result['cost'])
        print("Nodes Expanded:", result['nodes_expanded'])
        print("Max Frontier Size:", result['max_frontier_size'])
        print(f"Execution Time: {elapsed:.6f} seconds")
    else:
        print(f"\n{name}: No path found.")

# Drawing graph
show_graph = True
if show_graph:
    G = nx.Graph()
    for node, neighbors in graph_data.items():
        for neighbor, cost in neighbors.items():
            G.add_edge(node, neighbor, weight=cost)
    pos = nx.spring_layout(G, seed=42)
    edge_labels = nx.get_edge_attributes(G, 'weight')
    plt.figure(figsize=(10, 8))
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=800, font_weight='bold', font_size=12)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='red')
    plt.title("Network Graph for Packet Routing", fontsize=14)
    plt.axis('off')
    plt.show()
