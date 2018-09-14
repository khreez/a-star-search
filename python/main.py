from math import sqrt
import heapq


# Grid data structure is represented as a two dimensional array
# but it can also be thought as a graph where each node is linked to at most four adjacent nodes and
# where each link has a default weight of 1.
# Each grid element is defined as a tuple called `point` intersecting the x & y axis.
# Each point can access to its adjacent neighbours and each one has an associated cost if visited (going over the link)
# Only walls needs to be defined, otherwise a point is assumed to be walkable if is within boundaries
class Grid:
    def __init__(self, width, height):
        self.width = width  # size of the x axis
        self.height = height  # size of the y axis
        self.walls = []
        self.costs = {}

    # decides if a point is walkable, meaning is not a wall and is within bounds
    def is_walkable(self, point):
        (x, y) = point
        return point not in self.walls and (0 <= x < self.width and 0 <= y < self.height)

    # returns a list of adjacent points that are walkable
    def neighbors(self, point):
        (x, y) = point
        adjacent = [(x - 1, y), (x, y + 1), (x + 1, y), (x, y - 1)]  # get points adjacent to the given point
        return filter(self.is_walkable, adjacent)  # trim not walkable points

    # returns the associated cost of a given point
    def cost(self, point):
        return self.costs.get(point, 1)  # defaults to 1 if no cost is associated


# heuristic defined as the euclidean squared distance
# determines a cost between `origin` and `destination`
def heuristic(origin, destination):
    (x1, y1) = origin
    (x2, y2) = destination
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


# grid: holds a collection of points
# start: a tuple representing the point of start of the path
# goal: a tuple representing the goal point in the path
def a_star_search(grid, start, goal):
    frontier = []  # holds points to visit
    heapq.heappush(frontier, (0, start))  # starting point is added to the frontier with the lowest priority

    # holds the walked path, each point references the point of origin (where it came from)
    # initialized with the starting point as None due to being the first step
    path = {start: None}

    # holds the accumulated cost for a given visited point in the path
    accumulated_cost = {start: 0}

    # visit each point in the frontier until there's none left
    while not len(frontier) == 0:
        current = heapq.heappop(frontier)[1]  # pop point in frontier with the lowest priority

        if current == goal:
            break  # it has arrived to the goal, so stop

        # visit each neighbor for the given point in order to select the shortest/less expensive path
        for neighbor in grid.neighbors(current):
            # get the accumulated cost up to the current point plus the neighbor cost
            cost = accumulated_cost[current] + grid.cost(neighbor)

            # point not yet visited or the cost to visit is cheaper than the current one
            if neighbor not in accumulated_cost or cost < accumulated_cost[neighbor]:
                # incorporates the cost to visit the neighbor to the accumulated cost
                accumulated_cost[neighbor] = cost
                # given the cost to visit the neighbor, it needs to decide which point is the cheapest to visit next
                priority = cost + heuristic(neighbor, goal)
                # pushes the neighbor point into the priority queue, order will be decided based on the lowest priority
                heapq.heappush(frontier, (priority, neighbor))
                # adds current point to the visited path
                path[neighbor] = current

    return path


# helper function to print a grid and the resulting path
def draw_path(grid, start, goal, path):
    for y in range(grid.height - 1, -1, -1):
        for x in range(grid.width):
            value = draw_point(start, goal, grid.walls, path, (x, y))
            print("%%-%ds" % 2 % value, end="")
        print()


def draw_point(start, goal, walls, path, point):
    value = '.'  # walkable point
    if start == point:
        value = 'S'  # start point
    elif goal == point:
        value = 'G'  # goal point
    elif point in walls:
        value = '#'  # wall point
    elif point in path:
        value = '*'  # visited point
    return value


def main():
    # sample initial grid with roadblocks (walls)
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

    draw_path(grid, start, goal, path)


if __name__ == '__main__':
    main()
