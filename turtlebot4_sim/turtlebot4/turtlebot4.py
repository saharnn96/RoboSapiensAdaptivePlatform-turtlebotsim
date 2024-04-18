#**********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
import matplotlib.pyplot as plt

from turtlebot4_sim.turtlebot4.Controllers import *
from turtlebot4_sim.turtlebot4.plotting import *
from turtlebot4_sim.turtlebot4.Lidar import LaserSensor
from turtlebot4_sim.turtlebot4.Battery import Battery
from turtlebot4_sim.utils.helpers import AD2pos, perpetualTimer

import threading


class turtlebot4(object):

    def __init__(self,x_start=0.0,y_start=0.0,theta_start=0.0,showAnimation = True,verbose=False):
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
        self._show_animation = showAnimation

        #AT GOAL ALLOWED ERROR
        self._atGoalError = 0.001

        #robot parameters
        self._MAX_LINEAR_SPEED = 15
        self._MAX_ANGULAR_SPEED = 10
        self._robot_radius = 0.2  # [m]
        self._WB = 0.4  # [m] wheel base of vehicle

        # ROBOT SPEED
        self._target_speed = 2.0

        self._x = x_start
        self._y = y_start
        self._theta = theta_start
        self._v = 0.0

        # Environment and world map (obstacles, walls,...)
        self._environment = None
        self._map = None

        #move controller
        self._controller = PathFinderController(Kp_rho=9, Kp_alpha=15, Kp_beta=3)

        #pure pursuit controller
        self.Kp = 1.0  # speed proportional gain
        self.dt = 0.01  # [s] time tick

        # path planner
        grid_size = 2.0  # [m]
        self._planner = AStarPlanner(self._map, grid_size, self._robot_radius, show=False)
        self._waypoints = None

        # virtual lidar
        self._lidar = LaserSensor(position=[self._x, self._y], Range=120.0, gmap=self._map, uncertentity=(0.5, 0.01),angularResolution=6)  # angularResolution 6 instead of 1 for speedup
        self._lidar_executor = perpetualTimer(self._dt,self._lidar_callback)    #starts when map is provided

        # virtual battery
        self._battery = Battery()
        self._battery_executor = perpetualTimer(5, self._battery.spin_once) #execute every 5 seconds
        self._battery_executor.start()



    @property
    def target_speed(self):
        """The target_speed property (read-only)."""
        return self._target_speed

    @target_speed.setter
    def target_speed(self, value):
        """The target_speed property (write)."""
        self._target_speed = value


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
        return self._environment.map

    @map.setter
    def map(self,value):
        """The map property (write)."""
        self._map = value
        #give map to the virtual lidar
        self._lidar.map = self._map
        # spin the lidar using the new map
        self._lidar_executor.start()

    @property
    def environment(self):
        """The environment property (read-only)."""
        return self._environment
    @environment.setter
    def environment(self, value):
        """The map property (write)."""
        self._environment = value
        self._map = self._environment.map
        # give map to the virtual lidar
        self._lidar.map = self._environment.map
        # spin the lidar using the new map
        self._lidar_executor.start()


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

        # NEW PATH PLANNER
        path, path_px,rx,ry = a_star((self._x, self._y), (x_goal, y_goal), self.environment.restoredMap, movement='8N')
        rx = rx[::-1]
        ry = ry[::-1]

        self._waypoints = [rx, ry]

        return self._waypoints

    def navigate_waypoints(self,target_speed):
        self.target_speed = target_speed
        if self._waypoints is not None:
            states = States()
            state = State(x=self._x, y=self._y, yaw=self._theta, v=self._v,WB=self._WB,dt=0.1)  #TODO:dt generic!
            states.append(t=0.0,state=state)
            target_course = TargetCourse(self._waypoints[0], self._waypoints[1])
            target_ind, _ = target_course.search_target_index(state)

            lastIndex = len(self._waypoints[0]) - 1
            time=0.0
            while lastIndex > target_ind:
                time = time + self._dt   #TODO:dt generic!
                # Calc control input
                ai = self._proportional_control(self.target_speed, state.v)
                di, target_ind = self._pure_pursuit_steer_control(state, target_course, target_ind)

                state.update(ai, di)  # Control vehicle
                self._x, self._y, self._theta, self._v = state.x, state.y, state.yaw, state.v
                states.append(t=time, state=state)


                if self._show_animation:
                    self._plot()


        else:
            raise ValueError("No waypoints defined!")



    # ------------------------------------------------------------------------------------------------
    # --------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _exit(self):

        # --- stop all threads ---
        self._lidar_executor.cancel()
        self._battery_executor.cancel()

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

    def _lidar_callback(self):
        self._lidar.spin_once(self._x, self._y)

    def _onclick(self,event):
        self._show_animation = False
        self._exit()

    def _plot(self):

        plt.cla()

        # for stopping simulation on close event
        plt.gcf().canvas.mpl_connect('close_event', self._onclick)

        # --- formatting ---
        plt.grid(True)
        plt.title("Speed[km/h]:" + str(self._v * 3.6)[:4])

        # --- plot current world map ---
        self.map.plot()

        # --- plot turtlebot as arrow ---
        plot_arrow(self._x, self._y, self._theta)

        # --- plot trajectory ---
        plt.plot(self._waypoints[0], self._waypoints[1], "-r", label="course")

        # --- plot lidar pointcloud ---
        for point in self._lidar.cloudPoint:
            x, y = AD2pos(point[0], point[1], point[2])
            plt.plot([x, point[2][0]], [y, point[2][1]], 'b', linestyle="-")


        # --- refresh ---
        plt.pause(0.001)







