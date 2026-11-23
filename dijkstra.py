import heapq
from map import graph

def dijkstra(start):

    # initializes all distances to infinity and the start node to 0
    distances = {node: float('infinity') for node in graph}

    # stores where we came from
    previous = {}

    # distance from start to start is 0
    distances[start] = 0

    # priority queue (distance, node)
    queue = [(0, start)]

    while queue:

        # get the node with the smallest distance
        current_distance, current_node = heapq.heappop(queue)

        # skip if we found a better path already
        if current_distance > distances[current_node]:
            continue

        # check neighbors
        for neighbor, weight in graph[current_node].items():

            new_distance = current_distance + weight

            # check if we found a shorter route to the neighbor
            if new_distance < distances[neighbor]:

                # update distance
                distances[neighbor] = new_distance

                # update where we came from
                previous[neighbor] = current_node

                # add neighbor to the queue
                heapq.heappush(queue, (new_distance, neighbor))

    return distances, previous