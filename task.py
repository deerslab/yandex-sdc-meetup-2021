import sys
from collections import deque
import heapq

class Robot:

    def __init__(self, row, col, maze):
        self.row = row
        self.col = col
        self.maze = maze
        self.has_goods = False
        self.order = None

        self.f_row = None
        self.f_col = None
        self.unavailable_cords = []
        self.path = deque()


    def set_order(self, order, path):

        #path = self.get_path(order.s_row, order.s_col)

        if order:
            self.path = deque(path)
            self.order = order
            return True
        else:
            return False


    def astar(self, start, goal):
        def heuristic(start, goal):
            s_row, s_col = start
            f_row, f_col = goal
            return abs(s_row - f_row) + abs(s_col - f_col)

        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]

        close_set = set()
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}
        o_heap = []

        heapq.heappush(o_heap, (f_score[start], start))

        while o_heap:

            current = heapq.heappop(o_heap)[1]

            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                return data[::-1]

            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = g_score[current] + heuristic(current, neighbor)
                if 0 <= neighbor[0] < len(self.maze):
                    if 0 <= neighbor[1] < len(self.maze):
                        if self.maze[neighbor[0]][neighbor[1]] == 1:
                            continue
                    else:
                        # array bound y walls
                        continue
                else:
                    # array bound x walls
                    continue

                if neighbor in close_set and tentative_g_score >= g_score.get(neighbor, 0):
                    continue

                if tentative_g_score < g_score.get(neighbor, 0) or neighbor not in [
                    i[1] for i in o_heap
                ]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(o_heap, (f_score[neighbor], neighbor))

        return False


    def get_path(self, f_row, f_col):
        path = self.astar((self.row, self.col), (f_row, f_col))

        if not path:
            return None

        path = deque(path)

        self.f_row = f_row
        self.f_col = f_col
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

        iter_result = []
        for _ in range(60):
            for robot in robots:
                if not robot.order:
                    nearest_order, path = robot.get_nearest_order(orders)
                    if nearest_order:
                        order = orders.release_order(*nearest_order)
                        robot.set_order(order, path)

                iter_result.append(robot.tick())

        print("".join(iter_result), flush=True)


if __name__ == "__main__":
    run()
