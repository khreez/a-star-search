from math import sqrt
import heapq


class Grid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []
        self.costs = {}

    # decides if a point is walkable, meaning is not a wall and is within bounds
    def is_walkable(self, point):
        (x, y) = point
        return point not in self.walls and (0 <= x < self.width and 0 <= y < self.height)

    # returns a list of adjacent points that are walkable
    def neighbors(self, point):
        (x, y) = point
        adjacent = [(x - 1, y), (x, y + 1), (x + 1, y), (x, y - 1)]  # get clockwise points
        return filter(self.is_walkable, adjacent)  # trim not walkable points

    # returns the associated cost of a given point
    def cost(self, point):
        return self.costs.get(point, 1)  # defaults to 1 if no cost is associated


# euclidean squared distance
def heuristic(origin, destination):
    (x1, y1) = origin
    (x2, y2) = destination
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


# grid: holds tuples that represents a point or wall in a path
# start: tuple representing the point of start of the path
# goal: tuple representing the goal point in the path
def a_star_search(grid, start, goal):
    frontier = []  # holds tuples to visit
    heapq.heappush(frontier, (0, start))  # adding starting point to the frontier to visit as the lowest priority

    # holds the walked path, each point references the point of origin (where it came from)
    # initialized with the starting point as None due to being the first step
    path = {start: None}

    # holds the accumulated cost for a given visited point in the path
    accumulated_cost = {start: 0}

    # visit each point in the frontier until there's none
    while not len(frontier) == 0:
        current = heapq.heappop(frontier)[1]  # pop point in frontier with the lowest priority

        if current == goal:
            break  # it has arrived to the goal, so stop

        for neighbor in grid.neighbors(current):  # visit each neighbor for the given point
            # get the accumulated cost up to the current point plus the neighbor cost
            cost = accumulated_cost[current] + grid.cost(neighbor)

            if neighbor not in accumulated_cost or cost < accumulated_cost[neighbor]:
                accumulated_cost[neighbor] = cost
                priority = cost + heuristic(neighbor, goal)
                heapq.heappush(frontier, (priority, neighbor))
                path[neighbor] = current

    return path


def main():
    #  -----------
    # 4|. . . . .
    # 3|. . . # G
    # 2|. . . # .
    # 1|. # # # .
    # 0|S . . . .
    #  ----------
    #   0 1 2 3 4

    grid = Grid(5, 5)
    grid.walls = [(1, 1), (2, 1), (3, 1), (3, 2), (3, 3)]

    start = (0, 0)
    goal = (4, 3)

    path = a_star_search(grid, start, goal)

    for y in range(grid.height - 1, -1, -1):
        for x in range(grid.width):
            point = (x, y)

            value = '.'
            if start == point:
                value = 'S'
            elif goal == point:
                value = 'G'
            elif point in grid.walls:
                value = '#'
            elif point in path:
                value = 'X'

            print("%%-%ds" % 2 % value, end="")
        print()


if __name__ == '__main__':
    main()
