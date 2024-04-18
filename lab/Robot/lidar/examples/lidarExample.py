from lab.Robot.lidar.gridmap import OccupancyGridMap
import matplotlib.pyplot as plt
import math
from lab.Robot.lidar.a_star import a_star
from lab.Robot.lidar.utils import plot_path
from lab.Robot.lidar.lidar import LaserSensor


def AD2pos(distance, angle, robotposition):
    x = distance * math.cos(angle) + robotposition[0]
    y = -distance * math.sin(angle) + robotposition[1]
    return (int(x), int(y))

if __name__ == '__main__':
    # load the map
    gmap = OccupancyGridMap.from_png('maps/example_map_binary.png', 1)



    # set a start and an end node (in meters)
    start_node = (360.0, 330.0)
    goal_node = (285.0, 86.0)

    # run A*
    path, path_px = a_star(start_node, goal_node, gmap, movement='8N')
    print(path)
    gmap.plot()

    if path:
        # plot resulting path in pixels over the map
        plot_path(path_px)
    else:
        print('Goal is not reachable')

        # plot start and goal points over the map (in pixels)
        start_node_px = gmap.get_index_from_coordinates(start_node[0], start_node[1])
        goal_node_px = gmap.get_index_from_coordinates(goal_node[0], goal_node[1])

        plt.plot(start_node_px[0], start_node_px[1], 'ro')
        plt.plot(goal_node_px[0], goal_node_px[1], 'go')

    # --- lidar sensor ---
    _show_animation = True
    lidar = LaserSensor(position=[360.0, 330.0], Range=100.0, gmap=gmap, uncertentity=(0.5, 0.01),angularResolution=6)  #angularResolution 6 instead of 1 for speedup

    for position in path_px:
        lidar.position = [position[0], position[1]]
        cloud = lidar.sense_obstacles()
        for point in cloud:
            print(point)

        if _show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            gmap.plot()
            for point in cloud:
                x,y = AD2pos(point[0],point[1],point[2])
                plt.plot([x,point[2][0]], [y,point[2][1]], 'r', linestyle="-")

            plt.pause(0.001)

        #plt.pause(0.001)
    #plt.show()

