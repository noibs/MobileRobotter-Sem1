from dijkstra import dijkstra
from nearest_neighbor import nearest_neighbor

def print_route(start, dropoffs, destination=None):
    stops = [start, *dropoffs]
    if destination is not None:
        stops.append(destination)

    for leg_start, leg_end in zip(stops, stops[1:]):
        distances, previous = dijkstra(leg_start)
        path = [leg_end]

        while path[-1] != leg_start:
            path.append(previous[path[-1]])

        path.reverse()
        print(f"{' -> '.join(path)} (distance: {distances[leg_end]})")


white_dropoffs = [
    "WD1", "WD2", "WD3", "WD4", "WD5"
]

order = nearest_neighbor("P1", white_dropoffs)

print("Order of drop-offs:", order)
print("Full route including intersections:")
print_route("P1", order, "P2")

print("|--------------------------------------------------|\n")

red_dropoffs = [
    "RD1", "RD2", "RD3", "RD4", "RD5"
]

order = nearest_neighbor("P2", red_dropoffs)

print("Order of drop-offs:", order)
print("Full route including intersections:")
print_route("P2", order)