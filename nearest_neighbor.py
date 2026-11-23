from dijkstra import dijkstra

def nearest_neighbor(current_node, dropoffs):

    # create a copy of the dropoffs list to keep track of remaining nodes
    remaining_nodes = dropoffs.copy()
    order = []

    while remaining_nodes:

        # calculate distances from the current node to all remaining nodes
        distances, _ = dijkstra(current_node)

        # find the nearest neighbor - lambda function is used to get the distance for each node instead of the node itself
        nearest_node = min(remaining_nodes, key=lambda node: distances[node])

        # add the nearest node to the order
        order.append(nearest_node)

        # remove it from the remaining nodes
        remaining_nodes.remove(nearest_node)

        # update the current node
        current_node = nearest_node

    return order