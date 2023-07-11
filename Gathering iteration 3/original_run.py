import argparse
from functools import partial
import threading
from matplotlib import animation
import matplotlib.pyplot as plt
from threading import Thread, Lock, Barrier
from orginal_my_robot import *
import random
import math
import time

def parse_arguments():
    parser = argparse.ArgumentParser(description='Robot Simulation')
    parser.add_argument('--num_robots', type=int, default=20, help='Number of robots')
    parser.add_argument('--observation_radius', type=float, default=50, help='The visibility range of the robots')
    parser.add_argument('--speed', type=float, default=10, help='Speed of the robots')
    parser.add_argument('--width', type=int, default=100, help='Width of the simulation area')
    parser.add_argument('--height', type=int, default=100, help='Height of the simulation area')
    parser.add_argument('--num_diasbled_robots', type=int, default=3, help='Number of disabled robots')
    parser.add_argument('--output', type=str, default='animation.gif', help='Output file path for the animation')
    return parser.parse_args()

def is_visibility_graph_connected(robots):
    # Create a set of visited robots
    visited = set()

    # Perform a depth-first search (DFS) starting from the first robot
    dfs(robots[0], visited, robots=robots)

    # Check if all robots have been visited
    return len(visited) == len(robots)

def dfs(robot, visited, robots):
    visited.add(robot)

    # Perform DFS on neighboring robots within the observation radius
    for other_robot in robot.observe(robots):
        if other_robot not in visited:
            dfs(other_robot, visited, robots)
TERMINATE_ALL = False
def main():
    args = parse_arguments()
    fig, ax = plt.subplots()
    ax.set_xlim(0, args.width)
    ax.set_ylim(0, args.height)
    ax.set_aspect('equal')

    # barrier = Barrier(args.num_robots + 1)  # Create a Barrier object to synchronize threads

    threads = []
    robotsArr = []
    scatterArr = []
    lock = Lock()
    

    # Initialize the robots
    for i in range(args.num_robots):
        x = random.randint(0, args.width)
        y = random.randint(0, args.height)
        is_disabled = False

        if i < args.num_diasbled_robots:
            is_disabled = True

        robot = Robot(id=i, observation_radius=args.observation_radius, speed=args.speed,
                      x=x, y=y, terminate=False, trajectory_x=[x], trajectory_y=[y], disable = is_disabled)
        print("Robot ID" + str(robot.id) + "has been created")
        robotsArr.append(robot)
        scatterArr.append(ax.scatter(x, y))
    # Check visibility graph connectivity
    while not is_visibility_graph_connected(robotsArr):
        # Reset the robot positions until a connected visibility graph is achieved
        for robot in robotsArr:
            robot.x = random.randint(0, args.width)
            robot.y = random.randint(0, args.height)
    # barrier = Barrier(args.num_robots + 1)  # Create a Barrier object to synchronize threads

    # Start the threads
    for robot in robotsArr:
        thread = Thread(target=simulation, args=(robot, robotsArr, args))
        threads.append(thread)


    for thread in threads:
        thread.start()

    # Wait for all threads to reach the barrier
    # barrier.wait()  

    for thread in threads:
        thread.join()


    # for i in range(args.num_diasbled_robots, len(threads)):
    #     thread = threads[i]
    #     if thread.is_alive():
    #         print("Thread with ID", i, "is still running. Cannot terminate threads with smaller IDs.")
    #         break
    # else:
    # # All other threads with bigger IDs are terminated, terminate threads with smaller IDs
    #     for i in range(args.num_diasbled_robots):
    #         thread = threads[i]
    #         if thread.is_alive():
    #             print("Terminating thread with ID", i)
    #             thread.terminate()  # Set the terminate flag to True 

    ani = animation.FuncAnimation(fig, partial(animate, ls_of_scatter = scatterArr,ls_of_robots = robotsArr), repeat=True,
                                    frames=len(robotsArr[0].trajectory_x) - 1, interval=100)
    ani.save(args.output, writer='pillow')

    plt.show()

# movement_lock = Lock()

def simulation(robot, robots, args):
    global TERMINATE_ALL
    FULLY_CREATED = False
    while not robot.terminate and not TERMINATE_ALL:

        start = time.time()
        visible_robots = robot.observe(robots)
        # print(robot.id, "observing:", *[r.id for r in visible_robots], sep=', ')
        robot.move(visible_robots, robots)
        elapsed_time = time.time() - start

        if elapsed_time < 1:
            time.sleep(1 - elapsed_time)
        num_running_threads = threading.active_count() - 1
        # print("Number of running threads:", num_running_threads, "number of disabled robots:", args.num_diasbled_robots)
        if num_running_threads == len(robots):
            FULLY_CREATED = True
        if num_running_threads <= args.num_diasbled_robots and FULLY_CREATED:
            TERMINATE_ALL = True
            print("Robot", robot.id, "is forced to terminate")
    print("Robot", robot.id, "is done")


def animate(frame, ls_of_scatter, ls_of_robots):
    track_lens = []
    for robot in ls_of_robots:
        track_lens.append(len(robot.trajectory_x))
    min_track_len = min(track_lens)

    # for idx, scatter in enumerate(ls_of_scatter):
    #     robot = ls_of_robots[idx]

    #     if robot.disable and frame >= len(robot.trajectory_x):
    #         x = robot.x
    #         y = robot.y
    #     else:
    #         x = robot.trajectory_x[frame]
    #         y = robot.trajectory_y[frame]

    #     scatter.set_offsets((x, y))

    for idx, scatter in enumerate(ls_of_scatter):
        x = ls_of_robots[idx].trajectory_x[frame] if (lx := len(ls_of_robots[idx].trajectory_x)) > frame else ls_of_robots[idx].trajectory_x[lx-1]
        y = ls_of_robots[idx].trajectory_y[frame] if (ly := len(ls_of_robots[idx].trajectory_y)) > frame else ls_of_robots[idx].trajectory_y[ly-1]
        scatter.set_offsets((x, y))
    return scatter,

if __name__ == '__main__':
    main()