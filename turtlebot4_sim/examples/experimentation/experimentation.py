from turtlebot4_sim.environment.environment import Person
from turtlebot4_sim.environment.environment import Environment


# --- load the environment ---
environment = Environment(path_to_png="input/World_simple.png")

# --- load the person to walk in the environment
person = Person(x_start=200.0,y_start=50.0,theta_start=0.0,x_goal=150.0,y_goal=200.0,theta_goal=3.14)
person.environment = environment

# --- start the walk ---
person.start()