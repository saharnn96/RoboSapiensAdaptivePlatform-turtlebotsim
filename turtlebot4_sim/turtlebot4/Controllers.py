import matplotlib.pyplot as plt
import numpy as np
import math
from heapq import heappush, heappop
from turtlebot4_sim.utils.helpers import dist2d
from turtlebot4_sim.utils.angles import angle_mod


class PathFinderController:
    """
    Constructs an instantiate of the PathFinderController for navigating a
    3-DOF wheeled robot on a 2D plane

    Parameters
    ----------
    Kp_rho : The linear velocity gain to translate the robot along a line
             towards the goal
    Kp_alpha : The angular velocity gain to rotate the robot towards the goal
    Kp_beta : The offset angular velocity gain accounting for smooth merging to
              the goal angle (i.e., it helps the robot heading to be parallel
              to the target angle.)
    """

    def __init__(self, Kp_rho, Kp_alpha, Kp_beta):
        self.Kp_rho = Kp_rho
        self.Kp_alpha = Kp_alpha
        self.Kp_beta = Kp_beta

    def calc_control_command(self, x_diff, y_diff, theta, theta_goal):
        """
        Returns the control command for the linear and angular velocities as
        well as the distance to goal

        Parameters
        ----------
        x_diff : The position of target with respect to current robot position
                 in x direction
        y_diff : The position of target with respect to current robot position
                 in y direction
        theta : The current heading angle of robot with respect to x axis
        theta_goal: The target angle of robot with respect to x axis

        Returns
        -------
        rho : The distance between the robot and the goal position
        v : Command linear velocity
        w : Command angular velocity
        """

        # Description of local variables:
        # - alpha is the angle to the goal relative to the heading of the robot
        # - beta is the angle between the robot's position and the goal
        #   position plus the goal angle
        # - Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards
        #   the goal
        # - Kp_beta*beta rotates the line so that it is parallel to the goal
        #   angle
        #
        # Note:
        # we restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

        rho = np.hypot(x_diff, y_diff)
        alpha = angle_mod(np.arctan2(y_diff, x_diff) - theta)
        beta = angle_mod(theta_goal - theta - alpha)
        v = self.Kp_rho * rho
        w = self.Kp_alpha * alpha - self.Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        return rho, v, w

class AStarPlanner:

    def __init__(self, map, resolution, rr,show):
        """
        Initialize grid map for a star planning

        map: [ox,oy] [x position list of Obstacles [m],y position list of Obstacles [m]]
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        show: show the planner actions
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        if map is not None:
            self._map = map
            self.calc_obstacle_map(map[0], map[1])
        else:
            self._map = None

        self.show = show

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if self.show:  # pragma: no cover
                plt.plot(self._map[0], self._map[1], ".k")
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                #print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        self._map = [ox,oy]

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        #print("min_x:", self.min_x)
        #print("min_y:", self.min_y)
        #print("max_x:", self.max_x)
        #print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        #print("x_width:", self.x_width)
        #print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None
        # Parameters
        self.k = 0.1  # look forward gain
        self.Lfc = 2.0  # [m] look-ahead distance


    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = self.k * state.v + self.Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0,WB=2.9,dt=0.01):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
        self.WB = WB
        self.dt = dt

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw += self.v / self.WB * math.tan(delta) * self.dt
        self.v += a * self.dt
        self.rear_x = self.x - ((self.WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)

class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)





def _get_movements_4n():
    """
    Get all possible 4-connectivity movements.
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0)]


def _get_movements_8n():
    """
    Get all possible 8-connectivity movements. Equivalent to get_movements_in_radius(1).
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    s2 = math.sqrt(2)
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (1, 1, s2),
            (-1, 1, s2),
            (-1, -1, s2),
            (1, -1, s2)]


def a_star(start_m, goal_m, gmap, movement='8N', occupancy_cost_factor=3):
    """
    A* for 2D occupancy grid.

    :param start_m: start node (x, y) in meters
    :param goal_m: goal node (x, y) in meters
    :param gmap: the grid map
    :param movement: select between 4-connectivity ('4N') and 8-connectivity ('8N', default)
    :param occupancy_cost_factor: a number that will be multiplied by the occupancy probability
        of a grid map cell to give the additional movement cost to this cell (default: 3).

    :return: a tuple that contains: (the resulting path in meters, the resulting path in data array indices)
    """

    # get array indices of start and goal
    start = gmap.get_index_from_coordinates(start_m[0], start_m[1])
    goal = gmap.get_index_from_coordinates(goal_m[0], goal_m[1])

    # check if start and goal nodes correspond to free spaces
    if gmap.is_occupied_idx(start):
        raise Exception('Start node is not traversable')

    if gmap.is_occupied_idx(goal):
        raise Exception('Goal node is not traversable')

    # add start node to front
    # front is a list of (total estimated cost to goal, total cost from start to node, node, previous node)
    start_node_cost = 0
    start_node_estimated_cost_to_goal = dist2d(start, goal) + start_node_cost
    front = [(start_node_estimated_cost_to_goal, start_node_cost, start, None)]

    # use a dictionary to remember where we came from in order to reconstruct the path later on
    came_from = {}

    # get possible movements
    if movement == '4N':
        movements = _get_movements_4n()
    elif movement == '8N':
        movements = _get_movements_8n()
    else:
        raise ValueError('Unknown movement')

    # while there are elements to investigate in our front.
    while front:
        # get smallest item and remove from front.
        element = heappop(front)

        # if this has been visited already, skip it
        total_cost, cost, pos, previous = element
        if gmap.is_visited_idx(pos):
            continue

        # now it has been visited, mark with cost
        gmap.mark_visited_idx(pos)

        # set its previous node
        came_from[pos] = previous

        # if the goal has been reached, we are done!
        if pos == goal:
            break

        # check all neighbors
        for dx, dy, deltacost in movements:
            # determine new position
            new_x = pos[0] + dx
            new_y = pos[1] + dy
            new_pos = (new_x, new_y)

            # check whether new position is inside the map
            # if not, skip node
            if not gmap.is_inside_idx(new_pos):
                continue

            # add node to front if it was not visited before and is not an obstacle
            if (not gmap.is_visited_idx(new_pos)) and (not gmap.is_occupied_idx(new_pos)):
                potential_function_cost = gmap.get_data_idx(new_pos)*occupancy_cost_factor
                new_cost = cost + deltacost + potential_function_cost
                new_total_cost_to_goal = new_cost + dist2d(new_pos, goal) + potential_function_cost

                heappush(front, (new_total_cost_to_goal, new_cost, new_pos, pos))

    # reconstruct path backwards (only if we reached the goal)
    path = []
    path_idx = []
    rx=[]
    ry = []
    if pos == goal:
        while pos:
            path_idx.append(pos)
            # transform array indices to meters
            pos_m_x, pos_m_y = gmap.get_coordinates_from_index(pos[0], pos[1])
            path.append((pos_m_x, pos_m_y))
            pos = came_from[pos]
            #rx + ry
            rx.append(pos_m_x)
            ry.append(pos_m_y)


        # reverse so that path is from start to goal.
        path.reverse()
        path_idx.reverse()

    return path, path_idx,rx,ry
