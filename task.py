import sys
from collections import deque
import heapq
import random

route_base = {}

class Robot:

    def __init__(self, row, col, maze):
        self.row = row
        self.col = col
        self.maze = maze
        self.has_goods = False
        self.order = None

        self.unavailable_cords = []
        self.path = deque()


    def update_route_base(self):
        if self.path and (len(self.path)>0):
            goal = self.path[-1]

            if (self.row, self.col) in route_base:
                d = route_base.get((self.row, self.col))
            else:
                d = {}
            d[goal] = list(self.path)

            route_base[(self.row, self.col)] = d


    def set_order(self, order, path):

        #path = self.get_path(order.s_row, order.s_col)

        if order:
            self.path = deque(path)
            self.order = order
            return True
        else:
            return False


    def astar(self, start, goal):
        DIRECTIONS = ((1, 0), (0, 1), (-1, 0), (0, -1))

        if start in route_base:
            if goal in route_base.get(start):

                return deque(route_base.get(start).get(goal))

        class Node:
            def __init__(self, x, y, cost=sys.maxsize, h=0, parent=None):
                self.x = x
                self.y = y
                self.cost = cost
                self.h = h
                self.parent = parent

            def update(self, new_parent, new_cost):
                self.parent = new_parent
                self.cost = new_cost

            def __repr__(self):
                return "Node(x={x}, y={y}, cost={cost}, h={h}, parent={parent})".format(**self.__dict__)

            @property
            def priority(self):
                return self.cost + self.h

            @property
            def pos(self):
                return self.x, self.y

            def __eq__(self, other):
                return self.x == other.x and self.y == other.y

            def __lt__(self, other):
                """This allows Node to be used in the priority queue directly"""
                return self.priority < other.priority

        def make_grid(n_rows, n_cols, value):
            """Make a n_rows x n_cols grid filled with an initial value"""
            return [[value for _ in range(n_cols)] for _ in range(n_rows)]

        def is_valid(x, y, x_max, y_max):
            """Check the bounds and free space in the map"""
            if 0 <= x < x_max and 0 <= y < y_max:
                return self.maze[x][y] == 0
            return False

        def heuristic(start, goal):
            s_row, s_col = start
            f_row, f_col = goal
            return abs(s_row - f_row) + abs(s_col - f_col)

        x_max, y_max = len(self.maze), len(self.maze[0])

        # None will later be used as sentinel for "no node here (yet)"
        nodes = make_grid(x_max, y_max, None)

        start_node = Node(*start, cost=0, h=heuristic(start, goal))
        nodes[start_node.x][start_node.y] = start_node
        goal_node = Node(*goal)
        nodes[goal_node.x][goal_node.y] = goal_node

        # openlist will be used a priority queue and has to be accessed using
        # heappush and heappop from the heapq module. The Node class was modified
        # to work well with this kind of datastructure.
        openlist = []
        heapq.heappush(openlist, start_node)

        found = False
        while not found:
            # get the node with the least overall cost (actual + heuristic)
            current = heapq.heappop(openlist)
            for direction in DIRECTIONS:
                # compute new coordinates
                x_n, y_n = current.x + direction[0], current.y + direction[1]
                if not is_valid(x_n, y_n, x_max, y_max):
                    continue
                # we have valid coordinates
                if nodes[x_n][y_n] is None:
                    nodes[x_n][y_n] = Node(
                        x_n, y_n, h=heuristic((x_n, y_n), goal)
                    )
                # the new cost is made up if the current cost + transition
                new_cost = nodes[current.x][current.y].cost + 1
                if new_cost < nodes[x_n][y_n].cost:
                    # cool, we have found a faster path to this node, let's update
                    # it's predecessor
                    nodes[x_n][y_n].update(current.pos, new_cost)
                    heapq.heappush(openlist, nodes[x_n][y_n])
                    if nodes[x_n][y_n] == goal_node:
                        # we're done, get out of here
                        found = True
                        break
            # openlist is empty and we have not bailed out with found. seems like
            # there is nothing we can do here
            if not openlist:
                return []

        # backtracking
        path = []
        current = goal_node
        # this is a little bit weird because I decided to store only the
        # coordinates instead of the parent itself. Why? Because repr(node) is way
        #  more readable that way ;-)
        while True:
            path.append(current.pos)
            if current.parent is not None:
                current = nodes[current.parent[0]][current.parent[1]]
            else:
                break
        # the path is built by backtracking from the goal, so we have to reverse it
        path = path[::-1]

        if len(path)>0:
            path = path[1:]


        if len(path)>1:
            if start in route_base:
                d = route_base.get(start)
            else:
                d = {}
            d[goal] = path
            route_base[start] = d

        path = deque(path)

        return path


    def get_path(self, f_row, f_col):
        path = self.astar((self.row, self.col), (f_row, f_col))

        if not path:
            return None

        return path


    def tick(self):
        if self.order:
            if self.path:
                step = self.path.popleft()
                next_row, next_col = step
                if self.row == next_row:
                    if next_col == self.col+1:
                        #go right
                        self.col = next_col
                        return 'R'
                    elif next_col == self.col-1:
                        #go left
                        self.col = next_col
                        return 'L'
                    else:
                        raise Exception()
                else:
                    if next_row == self.row+1:
                        #go down
                        self.row = next_row
                        return 'D'
                    elif next_row == self.row-1:
                        #go up
                        self.row = next_row
                        return 'U'
                    else:
                        raise Exception()

            else:
                if self.has_goods:
                    #release order
                    self.order = None
                    self.has_goods = False
                    return 'P'
                else:
                    #getting order
                    self.has_goods = True
                    self.path = self.get_path(self.order.f_row, self.order.f_col)
                    return 'T'

        else:
            return 'S'


    def get_nearest_order(self, orders):
        def l1_dist(s_row, s_col, f_row, f_col):
            return abs(s_row - f_row) + abs(s_col-f_col)

        nearest_order = None
        nearest_order_path = None
        nearest_order_dist = sys.maxsize
        path = None

        for order_cords in orders.get_orders_cords():
            if (self.row, self.col) == order_cords:
                return (self.row, self.col), []

            if order_cords in self.unavailable_cords:
                continue

            order_dist = l1_dist(self.row, self.col, *order_cords)

            if order_dist < nearest_order_dist:

                nearest_order = order_cords
                #nearest_order_path = path
                nearest_order_dist = order_dist

        if nearest_order:
            path = self.astar((self.row, self.col), nearest_order)
            if not path:
                self.unavailable_cords.append(nearest_order)
                return None, []

        return nearest_order, path


class Order:

    def __init__(self, s_row, s_col, f_row, f_col):
        self.s_row = s_row-1
        self.s_col = s_col-1
        self.f_row = f_row-1
        self.f_col = f_col-1


class Orders:
    def __init__(self):
        self.base = {}


    def add_order(self, order):
        cords = (order.s_row, order.s_col)
        if cords in self.base:
            local_orders = self.base.get(cords)
            local_orders.append(order)
            self.base[cords] = local_orders
        else:
            self.base[cords] = [order]


    def release_order(self, row, col):
        cords = (row, col)
        local_orders = self.base.get(cords)
        order = local_orders[0]
        if len(local_orders)>1:
            local_orders = local_orders[1:]
            self.base[cords] = local_orders
        else:
            del self.base[cords]

        return order


    def get_orders_cords(self):
        return [*self.base]


def get_robots(maze):

    total_robots = (len(maze)*len(maze))//5000
    total_robots = max(1, total_robots)
    total_robots = min(100, total_robots)

    robots = []

    for _ in range(total_robots):
        while _ in range(100):
            i = random.randint(0, len(maze)-1)
            j = random.randint(0, len(maze)-1)
            if maze[i][j] == 0:
                robot = Robot(i, j, maze)
                robots.append(robot)
                break

    if len(robots) == 0:
        for i in range(len(maze)//2, len(maze)):
            for j in range(len(maze)//2, len(maze)):
                if maze[i][j] == 0:
                    robot = Robot(i, j, maze)
                    return [robot]

        for i in range(len(maze)):
            for j in range(len(maze)):
                if maze[i][j] == 0:
                    robot = Robot(i, j, maze)
                    return [robot]
    else:
        return robots


def readline():
    line = sys.stdin.readline().split()
    line = [int(value) for value in line]
    if len(line)==1:
        return line[0]

    return line


def read_map(N):
    maze = []
    for i in range(N):
        line = sys.stdin.readline().rstrip()
        line = [0 if node=='.' else 1 for node in line]
        maze.append(line)

    #maze = np.array(maze)

    return maze


def run():
    N, maxTips, cost = readline()

    maze = read_map(N)

    T, D = readline()

    robots = get_robots(maze)
    print(len(robots), flush=True)

    for robot in robots:
        row, col = robot.row, robot.col
        print(row+1, col+1, flush=True)

    orders = Orders()
    for _ in range(T):
        orders_num = readline()
        for _ in range(orders_num):
            s_row, s_col, f_row, f_col = readline()
            order = Order(s_row, s_col, f_row, f_col)
            orders.add_order(order)

        for robot in robots:
            iter_result = []
            stop = False
            for _ in range(60):

                if stop:
                    iter_result.append('S')
                else:

                    if not robot.order:
                        nearest_order, path = robot.get_nearest_order(orders)
                        if nearest_order:
                            order = orders.release_order(*nearest_order)
                            robot.set_order(order, path)

                    result = robot.tick()
                    iter_result.append(result)
                    robot.update_route_base()
                    if result is 'S':
                        stop = True

            print("".join(iter_result), flush=True)


if __name__ == "__main__":
    run()
