from turtlebot4_sim.environment.environment import Environment
from turtlebot4_sim.turtlebot4.AdaptiveTurtlebot4 import adaptiveTurtlebot4
import math

# --- load the environment ---
environment = Environment(path_to_png="00_input/World_simple.png")

# --- load the turtlebot 4 robot ---
robot = adaptiveTurtlebot4(x_start=200.0,y_start=50.0,theta_start=3.14,config='lidarocclusion-config.yaml',verbose=False)
robot.environment = environment

# --- perform actions ---
while True:
    robot._lidar.activateOverlay(overlay=[[0.0, 0.5, 0.0], [2.0, 3.0, 0.0]])      #[[angle_min, angle_max,distance]]

    # initial waypoint
    robot.determine_waypoints(x_goal=150.0, y_goal=200.0)
    robot.navigate_waypoints()

    # ANOMALY TRIGGERING PERSON START
    # person_anomly.start()

    # second waypoint
    robot.determine_waypoints(x_goal=100.0, y_goal=250.0)
    robot.navigate_waypoints()

    # ANOMALY ON LIDAR SIGNAL
    robot._lidar.activateOverlay(overlay=[[1.0,2.0, 0.0]])

    # initial waypoint
    robot.determine_waypoints(x_goal=200.0, y_goal=50.0)
    robot.navigate_waypoints()



# --- stop turtlebot after mission ---
robot._exit()



