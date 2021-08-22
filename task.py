import sys
from collections import deque

class Robot:

    def __init__(self, row, col, maze):
        self.row = row
        self.col = col
        self.maze = maze
        self.has_goods = False
        self.order = None

        self.f_row = None
        self.f_col = None
        self.path = deque()


    def set_order(self, order):
        self.order = order

        self.path = self.get_path(order.s_row, order.s_col)


    def astar(self, start, end):
        """Returns a list of tuples as a path from the given start to the given end in the given maze"""

        class Node():
            """A node class for A* Pathfinding"""

            def __init__(self, parent=None, position=None):
                self.parent = parent
                self.position = position

                self.g = 0
                self.h = 0
                self.f = 0

            def __eq__(self, other):
                return self.position == other.position

        # Create start and end node
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, end)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = []

        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end
        while len(open_list) > 0:

            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Found the goal
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]  # Return reversed path

            # Generate children
            children = []
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # Adjacent squares

                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # Make sure within range
                if node_position[0] > (len(self.maze) - 1) or node_position[0] < 0 or node_position[1] > (
                        len(self.maze[len(self.maze) - 1]) - 1) or node_position[1] < 0:
                    continue

                # Make sure walkable terrain
                if self.maze[node_position[0]][node_position[1]] != 0:
                    continue

                # Create new node
                new_node = Node(current_node, node_position)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:

                # Child is on the closed list
                for closed_child in closed_list:
                    if child == closed_child:
                        continue

                # Create the f, g, and h values
                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                            (child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue

                # Add the child to the open list
                open_list.append(child)


    def get_path(self, f_row, f_col):
        path = self.astar((self.row, self.col), (f_row, f_col))
        path = path[1:]
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
        nearest_order_dist = len(self.maze)*2

        for order_cords in orders.get_orders_cords():
            order_dist = l1_dist(self.row, self.col, *order_cords)
            if order_dist < nearest_order_dist:
                nearest_order = order_cords
                nearest_order_dist = order_dist

        return nearest_order


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
    #print(len(robots), flush=True)

    for robot in robots:
        row, col = robot.row, robot.col
        #print(row+1, col+1, flush=True)

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
                    nearest_order = robot.get_nearest_order(orders)
                    if nearest_order:
                        order = orders.release_order(*nearest_order)
                        robot.set_order(order)

                iter_result.append(robot.tick())

        #print("".join(iter_result), flush=True)


if __name__ == "__main__":
    run()
