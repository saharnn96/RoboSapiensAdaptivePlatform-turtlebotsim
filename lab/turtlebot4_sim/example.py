import time

from lab.turtlebot4_sim.turtlebot4 import turtlebot4
from lab.turtlebot4_sim.worlds import hallway

# setup the world
world = hallway()


robot = turtlebot4(x_start=2.0,y_start=2.0,theta_start=0,verbose=True)
robot.map = world.map

#robot.navigate_to_position(x_goal=30.0,y_goal=50.0,theta_goal=5)
#robot.dock()


robot.determine_waypoints(x_goal=50.0, y_goal=50.0)
robot.navigate_waypoints(target_speed=1)     #targetspeed [m/s]

robot.determine_waypoints(x_goal=0.0, y_goal=30.0)
robot.navigate_waypoints(target_speed=1)     #targetspeed [m/s]


