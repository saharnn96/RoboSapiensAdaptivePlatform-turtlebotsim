from turtlebot4_sim.environment.environment import Environment
from turtlebot4_sim.turtlebot4.turtlebot4 import turtlebot4

# --- load the environment ---
environment = Environment(path_to_png="input/World_simple.png")

# --- load the turtlebot 4 robot ---
robot = turtlebot4(x_start=200.0,y_start=50.0,theta_start=3.14,verbose=True)
#robot.map = environment.map
robot.environment = environment

# --- perform actions ---
while True:
    robot.determine_waypoints(x_goal=150.0, y_goal=200.0)
    robot.navigate_waypoints(target_speed=5)     #targetspeed [m/s]

    robot.determine_waypoints(x_goal=100.0, y_goal=250.0)
    robot.navigate_waypoints(target_speed=5)     #targetspeed [m/s]

    robot.determine_waypoints(x_goal=200.0, y_goal=50.0)
    robot.navigate_waypoints(target_speed=5)  # targetspeed [m/s]



# --- stop turtlebot after mission ---
robot._exit()



