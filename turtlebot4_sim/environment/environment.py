#**********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
from turtlebot4_sim.environment.gridmap import OccupancyGridMap
from turtlebot4_sim.turtlebot4.Controllers import *
from turtlebot4_sim.utils.helpers import AD2pos, perpetualTimer
class Environment(object):

    def __init__(self,path_to_png):
        """Initialize an environment using a png image

                Parameters
                ----------
                path_to_png : string
                    path to the 2D groundfloor

        See Also
        --------
        ..

        Examples
        --------
        >> environment = environment(path_to_png="floor.png")

        """

        self._map = None
        self._map = OccupancyGridMap.from_png(path_to_png, 1)
        self._png = path_to_png


    @property
    def map(self):
        """The map property (read-only)."""
        return self._map

    @property
    def restoredMap(self):
        return OccupancyGridMap.from_png(self._png, 1) #RELOAD NEEDED FOR REUSE OF MAPPING


class Person(object):

    def __init__(self, x_start=0.0, y_start=0.0, theta_start=0.0,x_goal=0.0, y_goal=0.0, theta_goal=0.0,v=2.0,UID="ID_001", showAnimation=True,verbose=False):
        """Initialize an virtual person in the environment

           Parameters
           ----------
           x_start : float
               initial x position of the person

           y_start : float
               initial y position of the person


           theta_start : float
               initial theta position of the person

           x_goal : float
               initial x position of the person

           y_goal : float
               initial y position of the person


           theta_goal : float
               initial theta position of the person

           v : float, optional
                initial velocity of the person (m/s)


           showAnimation : bool
                Show component animation

           verbose : bool
                component verbose execution


           See Also
           --------
           ..

           Examples
           --------
           >> p = Person(path_to_png="floor.png")

           """

        self._x = x_start
        self._y = y_start
        self._theta = theta_start
        self._x_goal = x_goal
        self._y_goal = y_goal
        self._theta_goal = theta_goal
        self._dt = 0.1
        self._WB = 0.5
        self._v = v
        self.Kp = 1.0  # speed proportional gain

        # THE MAP TO DETERMINE THE PATH
        self._environment = None

        # INITIALIZE THE STATE AND COURSE
        self._waypoints = None
        self._target_course = None
        self._target_ind, _ = None,None
        self._states = States()
        self._state = State(x=self._x, y=self._y, yaw=self._theta, v=self._v, WB=self._WB, dt=self._dt)  # TODO:dt generic!
        self._states.append(t=0.0, state=self._state)

        # UNIQUE ID FOR VIRTUAL CAMERA
        self._UID = UID
        self._detectionBound = 10.0



        # PERSON NAVIGATION STEP EXECUTOR
        self._isRunning = False
        self._walk_executor = perpetualTimer(self._dt, self._navigationStep)  # execute every dt seconds

    @property
    def environment(self):
        """The environment property (read-only)."""
        return self._environment

    @environment.setter
    def environment(self, value):
        """The map property (write)."""
        self._environment = value
        self._map = self._environment.map
        # set the target course
        self._waypoints = self._determine_waypoints(x_goal=self._x_goal, y_goal=self._y_goal)
        self._target_course = TargetCourse(self._waypoints[0], self._waypoints[1])
        self._target_ind, _ = self._target_course.search_target_index(self._state)

    def start(self):
        if not self._isRunning:
            self._isRunning = True
            self._walk_executor.start()

    def _navigationStep(self):
        lastIndex = len(self._waypoints[0]) - 1
        time=0.0
        if lastIndex > self._target_ind:
            time = time + self._dt   #TODO:dt generic!
            # Calc control input
            ai = self._proportional_control(self._v, self._state.v)
            di, target_ind = self._pure_pursuit_steer_control(self._state, self._target_course, self._target_ind)

            self._state.update(ai, di)  # Control vehicle
            self._x, self._y, self._theta, self._v = self._state.x, self._state.y, self._state.yaw, self._state.v
            self._states.append(t=time, state=self._state)




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

    def _determine_waypoints(self, x_goal, y_goal):

        # NEW PATH PLANNER
        path, path_px,rx,ry = a_star((self._x, self._y), (x_goal, y_goal), self.environment.restoredMap, movement='8N')
        rx = rx[::-1]
        ry = ry[::-1]

        self._waypoints = [rx, ry]

        return self._waypoints