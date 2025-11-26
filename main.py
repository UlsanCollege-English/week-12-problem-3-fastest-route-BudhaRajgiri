import heapq

def fastest_route(graph, start, goal):
    """
    Compute the shortest path and cost between start and goal using Dijkstra's algorithm.
    Graph format: {node: [(neighbor, weight), ...]}.
    Returns (path, cost). If unreachable, returns ([], None).
    """
    if start not in graph or goal not in graph:
        return ([], None)

    if start == goal:
        return ([start], 0)

    dist = {node: float("inf") for node in graph}
    dist[start] = 0
    parent = {start: None}
    pq = [(0, start)]

    while pq:
        cost, node = heapq.heappop(pq)
        if cost > dist[node]:
            continue
        for neighbor, weight in graph.get(node, []):
            new_cost = cost + weight
            if new_cost < dist[neighbor]:
                dist[neighbor] = new_cost
                parent[neighbor] = node
                heapq.heappush(pq, (new_cost, neighbor))

    if dist[goal] == float("inf"):
        return ([], None)

    # Reconstruct path
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = parent.get(node)
    path.reverse()

    return (path, dist[goal])