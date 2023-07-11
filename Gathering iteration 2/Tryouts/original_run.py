import argparse
from functools import partial
from matplotlib import animation
import matplotlib.pyplot as plt
from threading import Thread, Lock, Barrier
# from robots import *
from orginal_my_robot import *
import random
import math
import time

def parse_arguments():
    parser = argparse.ArgumentParser(description='Robot Simulation')
    parser.add_argument('--num_robots', type=int, default=20, help='Number of robots')
    parser.add_argument('--observation_radius', type=float, default=35, help='The visibility range of the robots')
    parser.add_argument('--speed', type=float, default=10, help='Speed of the robots')
    parser.add_argument('--width', type=int, default=100, help='Width of the simulation area')
    parser.add_argument('--height', type=int, default=100, help='Height of the simulation area')
    parser.add_argument('--num_diasbled_robots', type=int, default=3, help='Number of disabled robots')
    return parser.parse_args()

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

    # barrier = Barrier(args.num_robots + 1)  # Create a Barrier object to synchronize threads

    # Start the threads
    for robot in robotsArr:
        thread = Thread(target=simulation, args=(robot, robotsArr))
        threads.append(thread)

    for thread in threads:
        thread.start()

    # Wait for all threads to reach the barrier
    # barrier.wait()  

    for thread in threads:
        thread.join()
    ani = animation.FuncAnimation(fig, partial(animate, ls_of_scatter = scatterArr,ls_of_robots = robotsArr), repeat=True,
                                    frames=len(robotsArr[0].trajectory_x) - 1, interval=100)
    plt.show()

# movement_lock = Lock()


def simulation(robot, robots):
    while not robot.terminate:
        start = time.time()

        visible_robots = robot.observe(robots)
        print(robot.id, "observing:", *[r.id for r in visible_robots], sep=', ')
        robot.move(visible_robots, robots)
        elapsed_time = time.time() - start

        if elapsed_time < 1:
            time.sleep(1 - elapsed_time)

    print("Robot", robot.id, "is done")

def animate(frame, ls_of_scatter, ls_of_robots):
    track_lens = []
    for robot in ls_of_robots:
        track_lens.append( len(robot.trajectory_x))
    min_track_len = min(track_lens)

    for idx, scatter in enumerate(ls_of_scatter):
        x = ls_of_robots[idx].trajectory_x[frame] if (lx := len(ls_of_robots[idx].trajectory_x)) > frame else ls_of_robots[idx].trajectory_x[lx-1]
        y = ls_of_robots[idx].trajectory_y[frame] if (ly := len(ls_of_robots[idx].trajectory_y)) > frame else ls_of_robots[idx].trajectory_y[ly-1]
        scatter.set_offsets((x, y))
    return scatter,

if __name__ == '__main__':
    main()