from turtlebot4_sim.environment.environment import Environment,Person
from turtlebot4_sim.turtlebot4.AdaptiveTurtlebot4 import adaptiveTurtlebot4

# --- load the environment ---
environment = Environment(path_to_png="00_input/World_simple.png")

# --- load persons walking in the environment
person_anomly = Person(x_start=100.0,y_start=250.0,theta_start=0.0,x_goal=150.0,y_goal=200.0,theta_goal=3.14,v=3.0,UID="PERSON_ANOMALY")    # HEADING TOWARDS THE ROBOT = ANOMALY
person_anomly.environment = environment

person_normal = Person(x_start=100.0,y_start=250.0,theta_start=0.0,x_goal=200.0,y_goal=150.0,theta_goal=3.14,v=2.5,UID="PERSON_NORMAL1")    # PASSING THE ROBOT = NORMAL
person_normal.environment = environment

# --- load the turtlebot 4 robot ---
robot = adaptiveTurtlebot4(x_start=200.0,y_start=50.0,theta_start=3.14,config='input/config.yaml',verbose=False)
robot.environment = environment
robot._personList.append(person_anomly)
robot._personList.append(person_normal)

# NORMAL PERSON START
person_normal.start()

# --- perform actions ---
while True:

    # initial waypoint
    robot.determine_waypoints(x_goal=150.0, y_goal=200.0)
    robot.navigate_waypoints()

    # ANOMALY TRIGGERING PERSON START
    person_anomly.start()

    # second waypoint
    robot.determine_waypoints(x_goal=100.0, y_goal=250.0)
    robot.navigate_waypoints()

    # initial waypoint
    robot.determine_waypoints(x_goal=200.0, y_goal=50.0)
    robot.navigate_waypoints()



# --- stop turtlebot after mission ---
robot._exit()



