from turtlebot4_sim.environment.environment import Environment,Person
from turtlebot4_sim.turtlebot4.AdaptiveTurtlebot4 import adaptiveTurtlebot4

# --- load the environment ---
environment = Environment(path_to_png="00_input/World_simple.png")

# --- load the turtlebot 4 robot ---
robot = adaptiveTurtlebot4(x_start=200.0,y_start=50.0,theta_start=3.14,config='00_input/config.yaml',verbose=False)
robot.environment = environment


# --- perform actions ---
while True:


    # initial waypoint
    robot.determine_waypoints(x_goal=150.0, y_goal=200.0)
    robot.navigate_waypoints()

    # ANOMALY TRIGGERING LIDAR OCCLUSION
    robot._lidar.activateOverlay(overlay=[[0.0, 0.5,5.0], [2.0, 3.0,10.0]])      #[[angle_min, angle_max,distance]]

    # second waypoint
    robot.determine_waypoints(x_goal=100.0, y_goal=250.0)
    robot.navigate_waypoints()

    # ANOMALY ON LIDAR SIGNAL
    robot._lidar.activateOverlay(overlay=[[0.0,0.5],[2.0,3.0]])

    # initial waypoint
    robot.determine_waypoints(x_goal=200.0, y_goal=50.0)
    robot.navigate_waypoints()



# --- stop turtlebot after mission ---
robot._exit()



