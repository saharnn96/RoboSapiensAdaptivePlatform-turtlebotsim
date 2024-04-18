#**********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
import warnings

import matplotlib.pyplot as plt
import numpy as np
import math
from random import random
from lab.turtlebot4_sim.utils.angles import angle_mod

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

def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

class turtlebot4(object):

    def __init__(self,x_start=0.0,y_start=0.0,theta_start=0.0,verbose=False):
        """Initialize the irobot create 3 instance

                Parameters
                ----------
                verbose : bool
                    component verbose execution

        See Also
        --------
        ..

        Examples
        --------
        >> robot = irobot_create_3(verbose=False)

        """

        self._name = "Irobot create 3 "
        self._description = "A simulation instance of the irobot create 3."
        self._verbose = verbose



        #sim parameters
        self._dt = 0.01
        self._show_animation = True

        #AT GOAL ALLOWED ERROR
        self._atGoalError = 0.001

        #robot parameters
        self._MAX_LINEAR_SPEED = 15
        self._MAX_ANGULAR_SPEED = 10
        self._robot_radius = 0.2  # [m]
        self._WB = 0.4  # [m] wheel base of vehicle

        self._x = x_start
        self._y = y_start
        self._theta = theta_start
        self._v = 0.0

        # world map (obstacles, walls,...)
        self._map = None

        #move controller
        self._controller = PathFinderController(Kp_rho=9, Kp_alpha=15, Kp_beta=3)

        #pure pursuit controller
        self.Kp = 1.0  # speed proportional gain
        self.dt = 0.1  # [s] time tick



        # path planner
        grid_size = 2.0  # [m]
        self._planner = AStarPlanner(self._map, grid_size, self._robot_radius, show=False)
        self._waypoints = None



    @property
    def name(self):
        """The name property (read-only)."""
        return self._name

    @property
    def description(self):
        """The description property (read-only)."""
        return self._description

    @property
    def map(self):
        """The map property (read-only)."""
        return self._map

    @map.setter
    def map(self,value):
        """The map property (write)."""
        self._map = value
        self._planner.calc_obstacle_map(ox=self._map[0],oy=self._map[1])    #recalculating obstacle map when new worldmap is provided

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERFACE FUNCTIONS---------------------------------------
    # ------------------------------------------------------------------------------------------------
    def dock(self):
        """Function to dock the robot."""
        self.navigate_to_position(x_goal=0.0,y_goal=0.0,theta_goal=3.14)

    def undock(self):
        """Function to undock the robot."""
        self.navigate_to_position(x_goal=0.0,y_goal=0.0,theta_goal=0)

    def navigate_to_position(self, x_goal, y_goal, theta_goal):
        x_init = self._x
        y_init = self._y
        theta_init = self._theta

        x_diff = x_goal - self._x
        y_diff = y_goal - self._y

        x_traj, y_traj = [], []

        rho = np.hypot(x_diff, y_diff)
        while rho > self._atGoalError:
            x_traj.append(self._x)
            y_traj.append(self._y)

            x_diff = x_goal - self._x
            y_diff = y_goal - self._y

            rho, v, w = self._controller.calc_control_command(
                x_diff, y_diff, self._theta, theta_goal)

            if abs(v) > self._MAX_LINEAR_SPEED:
                v = np.sign(v) * self._MAX_LINEAR_SPEED

            if abs(w) > self._MAX_ANGULAR_SPEED:
                w = np.sign(w) * self._MAX_ANGULAR_SPEED

            self._theta = self._theta + w * self._dt
            self._x = self._x + v * np.cos(self._theta) * self._dt
            self._y = self._y + v * np.sin(self._theta) * self._dt

            if self._show_animation:  # pragma: no cover
                plt.cla()
                plt.arrow(x_init, y_init, np.cos(theta_init),
                          np.sin(theta_init), color='r', width=0.5)
                plt.arrow(x_goal, y_goal, np.cos(theta_goal),
                          np.sin(theta_goal), color='g', width=0.5)
                #show world map
                plt.plot(self.map[0], self.map[1], ".k")
                self._plot_vehicle(self._x, self._y, self._theta, x_traj, y_traj)


    def determine_waypoints(self, x_goal, y_goal):

        rx, ry = self._planner.planning(self._x, self._y, x_goal, y_goal)
        rx = rx[::-1]
        ry = ry[::-1]
        self._waypoints = [rx,ry]
        print(self._waypoints)

        return self._waypoints

    def navigate_waypoints(self,target_speed):
        if self._waypoints is not None:
            states = States()
            state = State(x=self._x, y=self._y, yaw=self._theta, v=self._v,WB=self._WB,dt=0.1)  #TODO:dt generic!
            states.append(t=0.0,state=state)
            target_course = TargetCourse(self._waypoints[0], self._waypoints[1])
            target_ind, _ = target_course.search_target_index(state)

            lastIndex = len(self._waypoints[0]) - 1
            time=0.0
            while lastIndex > target_ind:
                time = time + 0.1   #TODO:dt generic!
                # Calc control input
                ai = self._proportional_control(target_speed, state.v)
                di, target_ind = self._pure_pursuit_steer_control(state, target_course, target_ind)

                state.update(ai, di)  # Control vehicle
                self._x, self._y, self._theta, self._v = state.x, state.y, state.yaw, state.v
                states.append(t=time, state=state)

                if self._show_animation:  # pragma: no cover
                    plt.cla()
                    # for stopping simulation with the esc key.
                    plt.gcf().canvas.mpl_connect(
                        'key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                    plot_arrow(state.x, state.y, state.yaw)
                    plt.plot(self._waypoints[0], self._waypoints[1], "-r", label="course")
                    plt.plot(states.x, states.y, "-b", label="trajectory")
                    plt.plot(self._waypoints[0][target_ind], self._waypoints[1][target_ind], "xg", label="target")
                    plt.axis("equal")
                    plt.grid(True)
                    plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
                    # show world map
                    plt.plot(self.map[0], self.map[1], ".k")
                    plt.pause(0.001)

        else:
            raise ValueError("No waypoints defined!")

    # ------------------------------------------------------------------------------------------------
    # --------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _proportional_control(self,target, current):
        a = self.Kp * (target - current)
        return a

    def _pure_pursuit_steer_control(self,state, trajectory, pind):
        ind, Lf = trajectory.search_target_index(state)

        if pind >= ind:
            ind = pind

        if ind < len(trajectory.cx):
            tx = trajectory.cx[ind]
            ty = trajectory.cy[ind]
        else:  # toward goal
            tx = trajectory.cx[-1]
            ty = trajectory.cy[-1]
            ind = len(trajectory.cx) - 1

        alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

        delta = math.atan2(2.0 * self._WB * math.sin(alpha) / Lf, 1.0)

        return delta, ind

    def _plot_vehicle(self,x, y, theta, x_traj, y_traj):  # pragma: no cover
        # Corners of triangular vehicle when pointing to the right (0 radians)
        p1_i = np.array([0.5, 0, 1]).T
        p2_i = np.array([-0.5, 0.25, 1]).T
        p3_i = np.array([-0.5, -0.25, 1]).T

        T = self._transformation_matrix(x, y, theta)
        p1 = np.matmul(T, p1_i)
        p2 = np.matmul(T, p2_i)
        p3 = np.matmul(T, p3_i)

        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
        plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
        plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

        plt.plot(x_traj, y_traj, 'b--')


        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        plt.xlim(-20, 65)
        plt.ylim(-20, 65)

        plt.pause(self._dt)


    def _transformation_matrix(self,x, y, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta), np.cos(theta), y],
            [0, 0, 1]
        ])