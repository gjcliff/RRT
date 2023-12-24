from matplotlib.collections import LineCollection
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import sympy as sym
import cv2

import argparse

import time


class RRT():
    def __init__(self, mode=0, num_obs=1):
        self.G = {}  # this is the graph that will hold all the nodes
        self.delta = 3  # this is the incremental distance
        self.D = (200, 200)  # this is the domain
        self.K = 10000

        self.mode = mode
        self.num_obs = num_obs

        self.precision = 10

    def RANDOM_CONFIGURATION(self):
        '''
        Generate a random position in the domain (D).

        Args:
        ----
        D (int tuple): the domain space.

        Returns:
        -------
        qRand (float tuple): a random position in the domain space.

        '''
        rng = np.random.default_rng()

        x = rng.random() * self.D[0]
        y = rng.random() * self.D[1]

        qRand = (x, y)

        return qRand

    def NEAREST_VERTEX(self, qRand):
        '''
        Finds the vertex in that is closest to the given position using the
        Euclidean metric.

        Args:
        ----
        qRand (list): a random position in the domain.
        G (list): the list of nodes in the domain.

        '''
        # longest possible distance
        distance = np.linalg.norm(np.asarray(self.D) - np.asarray((0, 0)))

        qNear = ()
        for k in self.G:
            distance_temp = np.linalg.norm(np.asarray(qRand) - np.asarray(k))
            if distance_temp < distance:
                distance = distance_temp
                qNear = k

        return qNear

    def NEW_CONFIGURATION(self, qNear, qRand):
        '''
        Generate a new node in the tree by moving a distance "delta" from node
        qNear towards qRand
        '''
        # generates a new configuraition in the tree by moving some distance,d
        # delta, from one vertex configuration towards another configuration

        # vector = np.subtract(qRand, qNear)
        vector = np.subtract(qRand, qNear)

        unitVector = vector / np.linalg.norm(vector)

        # qNew = tuple(np.add(qNear, np.multiply(unitVector, delta)))

        # have to cast to tuple type or else the object will be numpy array,
        # which is unhashable
        # and this element will be a key in the G dictionary.
        qNew = tuple(qNear + unitVector * self.delta)

        return qNew

    def draw_plots(self, qGoal):
        # this function creates the scatter plot, line collection, and defines
        # some of their attributes.

        plt.ion()  # turn on interactive mode
        fig, ax = plt.subplots(figsize=(16, 16))  # create the sub plot

        plot = ax.scatter([], [], c="blue", s=1)  # create the scatter plot
        goal = ax.scatter([qGoal[0]], [qGoal[1]], c="red", s=3)
        # goal.set_offsets(np.column_stack([,y])) # add the new x and y
        # coordiantes to the plot
        line_segments = LineCollection([], colors="blue", linestyle="solid")
        ax.add_collection(line_segments)
        ax.set_xlim(0, self.D[0])
        ax.set_ylim(0, self.D[1])

        plt.draw()
        plt.show()

        return fig, ax, plot, line_segments

    def update_plots(self, x, y, line_segments, lines, plot, fig, qNew, qNear):
        # this function is meant to be called in each iteration of the for loop
        # inside rrt_algo(). It updates the plot with new nodes as they're
        # generated.

        # adding the new point, and the point it was
        lines.append([qNear, qNew])

        # adding the new line to the plot
        line_segments.set_segments(lines)

        # add the x coordinate of the new point to the x coordinate list
        x.append(qNew[0])

        # add the y coordinate of the new point to the y coordinate list
        y.append(qNew[1])

        # add the new x and y coordiantes to the plot
        plot.set_offsets(
            np.column_stack([x, y]))

        # update the canvas
        fig.canvas.draw_idle()

    def test_slope(self, point1, point2):
        """
        Test whether the slope between the two points is infinity.

        If the slope between the two points is infinity, return False
        to indicate that the slope is invalid. If the slope is non-zero,
        then return True to indicate that the slope is valid.

        Args:
        ----
        point1 (tuple): The start point of a line segment.
        point2 (tuple): The end point of a line segment.

        """
        if point2[0] == point1[0]:
            return False
        else:
            return True

    def checkCollision(self, qThere, qHere, obstacles, flag):
        '''
        Check to see if there is a collision.

        Check to see if there is a collision with an obstacle between
        qThere and qHere, both of which are points in the graph.

        Args:
        ----
        qThere: Point that is somewhere else.
        qHere: Point that is right where we're located.
        obstacles: A list of the positions of obstacles.
        flag (int): whether or not the obstacle type is a matplotlib
        shape or an image.

        '''
        qLine = np.subtract(qThere, qHere)

        qDistance = np.linalg.norm(np.asarray(qThere) - np.asarray(qHere))

        if flag:

            for key in obstacles:
                # retrieve the center point and radius for the obstacle
                print(f"obstaclces[key]: {obstacles[key]}")
                x, y, r = obstacles[key]

                oDistance = np.linalg.norm(
                    np.asarray((x, y)) - np.asarray(qHere))

                # create a numpy array representing a vector between the center of
                # the circle and the new node
                oLine = np.subtract((x, y), qHere)

                dist = np.linalg.norm(np.cross(oLine, qLine)) / qDistance

                dot_product = np.dot(qLine, oLine)

                if dist < r and dot_product > 0 and qDistance > oDistance - r:
                    return True  # if a collision is found, return true

            return False  # if no collisions are found, return false

        else:
            for i in range(1, len(obstacles)):
                # print(f"i: {i}")

                point_slope_valid = self.test_slope(qHere, qThere)
                obstacle_slope_valid = self.test_slope(
                    obstacles[i], obstacles[i-1])
                x = sym.symbols('x')

                if not point_slope_valid and not obstacle_slope_valid:

                    # if both the obstacle slope and the slope between the
                    # points at qHere and qThere are vertical, then they
                    # do not intersect and we can move on to the next obstacle.

                    continue

                elif not obstacle_slope_valid:

                    # if the obstacle is a vertical line,find the x coordinate
                    # where that vertical line is and check whether or not the
                    # obstacle line intersetcts with it.

                    # the x position of the first and second point in the obstacle are the same
                    x = qHere[0]
                    y = qHere[1]
                    m = (qThere[1] - qHere[1])/(qThere[0] - qHere[0])

                    b = y - m * x

                    obstacle_x = obstacles[i-1][0]
                    obstacle_y = m * obstacle_x + b  # y = mx+b

                    if min(obstacles[i][1], obstacles[i-1][1]) <= round(obstacle_y, self.precision) <= max(obstacles[i][1], obstacles[i-1][1])\
                            and min(qThere[1], qHere[1]) <= round(obstacle_y, self.precision) <= max(qThere[1], qHere[1]):

                        return True

                    else:

                        continue

                elif not point_slope_valid:

                    x = obstacles[i-1][0]
                    y = obstacles[i-1][1]
                    m = (obstacles[i][1] - obstacles[i-1][1]) / (
                        obstacles[i][0] - obstacles[i-1][0])

                    b = y - m * x

                    qx = qHere[0]
                    qy = m * qx + b

                    if min(obstacles[i][1], obstacles[i-1][1]) <= round(qy, self.precision) <= max(obstacles[i][1], obstacles[i-1][1])\
                            and min(qThere[1], qHere[1]) <= round(qy, self.precision) <= max(qThere[1], qHere[1]):

                        return True

                    else:
                        continue

                elif (qThere[1] - qHere[1])/(qThere[0] - qHere[0]) == (obstacles[i-1][1] - obstacles[i][1])/(obstacles[i-1][0] - obstacles[i][0]):

                    # if the slopes of the obstacle and the line between qHere
                    # and qThere are equal, then they do not intersect and we
                    # can move on to the next obstacle.

                    continue

                x_obstacle = obstacles[i-1][0]
                y_obstacle = obstacles[i-1][1]
                m_obstacle = (obstacles[i][1] - obstacles[i-1][1]) / (
                    obstacles[i][0] - obstacles[i-1][0])

                b_obstacle = y_obstacle - m_obstacle * x_obstacle

                x_q = qHere[0]
                y_q = qHere[1]
                m_q = (qThere[1] - qHere[1])/(qThere[0] -
                                              qHere[0])

                b_q = y_q - m_q * x_q

                x = (b_obstacle - b_q)/(m_q - m_obstacle)
                y = m_q * x + b_q

                if min(qThere[0], qHere[0]) <= round(x, self.precision) <= max(qThere[0], qHere[0])\
                    and min(qThere[1], qHere[1]) <= round(y, self.precision) <= max(qThere[1], qHere[1])\
                        and min(obstacles[i][0], obstacles[i-1][0]) <= round(x, self.precision) <= max(obstacles[i][0], obstacles[i-1][0])\
                    and min(obstacles[i][1], obstacles[i-1][1]) <= round(y, self.precision) <= max(obstacles[i][1], obstacles[i-1][1]):

                    return True

            return False

    def randomStart(self, image=None):

        rng = np.random.default_rng()

        x, y = rng.integers(20, self.D[0] - 20, size=2)

        if self.mode:
            x, y = rng.integers(20, self.D[0] - 20, size=2)
        else:
            while image[y, x] == 0:
                x, y = rng.integers(20, self.D[0] - 20, size=2)

        qInit = (x, y)

        return qInit

    def randomGoal(self, image=None):
        rng = np.random.default_rng()

        x, y = rng.integers(20, self.D[0] - 20, size=2)

        if self.mode:
            x, y = rng.integers(20, self.D[0] - 20, size=2)
        else:
            while image[y, x] == 0:
                x, y = rng.integers(20, self.D[0] - 20, size=2)

        qGoal = (x, y)

        return qGoal

    def randomObstacles(self, ax, qInit, qGoal):

        rng = np.random.default_rng()  # add the random number generator

        numObstacles = self.num_obs
        circles = {}
        obstacles = {}

        for i in range(numObstacles):
            successful = False

            while not successful:
                r = rng.integers(0, 20)
                x = rng.integers(20, self.D[0] - 20)
                y = rng.integers(20, self.D[1] - 20)

                center = [x, y]
                startDist = np.linalg.norm(
                    np.asarray(center) - np.asarray(qInit))
                goalDist = np.linalg.norm(
                    np.asarray(center) - np.asarray(qGoal))

                if startDist > r and goalDist > r:
                    circles[f"circles{i}"] = Circle([x, y], r)
                    ax.add_patch(circles[f"circles{i}"])
                    obstacles[f"o{i}"] = [x, y, r]
                    successful = True
        return obstacles

    def load_image_binary(self):
        image = cv2.imread("pics/N_map.png", cv2.IMREAD_GRAYSCALE)

        image_scaled = cv2.resize(
            image, (self.D[0], self.D[1]), interpolation=cv2.INTER_NEAREST)

        image_flipped = cv2.flip(image_scaled, 1)

        contours, _ = cv2.findContours(
            image_flipped, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        contours = contours[0][:, 0, :]

        contours = np.vstack([contours, contours[0]])
        return contours, image_flipped

    def initialize_mode_one(self):

        qInit = self.randomStart()
        qGoal = self.randomGoal()

        fig, ax, plot, line_segments = self.draw_plots(qGoal)

        obstacles = self.randomObstacles(ax, qInit, qGoal)

        return qInit, qGoal, obstacles, fig, ax, plot, line_segments

    def initialize_mode_zero(self):
        obstacles, image_scaled = self.load_image_binary()
        qInit = self.randomStart(image_scaled)
        qGoal = self.randomGoal(image_scaled)

        fig, ax, plot, line_segments = self.draw_plots(qGoal)

        ax.imshow(image_scaled, cmap='gray', origin='upper')

        return qInit, qGoal, obstacles, image_scaled, fig, ax, plot, line_segments

    def draw_goal_path(self, qGoal, ax, fig):

        key = qGoal

        lines = []

        while self.G[key] is not None:

            point1 = key
            point2 = self.G[key]

            lines.append([point1, point2])

            key = point2

        line_segments = LineCollection(lines, colors="red", linestyle="solid")
        ax.add_collection(line_segments)
        fig.canvas.draw_idle()

    def rrt_algo(self):
        x, y, lines = [], [], []

        if self.mode:
            qInit, qGoal, obstacles, fig, ax, plot, line_segments = self.initialize_mode_one()
        else:
            qInit, qGoal, obstacles, image_scaled, fig, ax, plot, line_segments = self.initialize_mode_zero()

        self.G.update({qInit: None})
        for i in range(self.K):

            qRand = self.RANDOM_CONFIGURATION()
            qNear = self.NEAREST_VERTEX(qRand)
            qNew = self.NEW_CONFIGURATION(qNear, qRand)

            if self.mode:
                collision = self.checkCollision(
                    qNew, qNear, obstacles, self.mode
                )
            else:
                collision = False
                if image_scaled is not None and image_scaled[round(qNew[1]), round(qNew[0])] == 0:
                    collision = True

            if not collision:

                self.G.update({qNew: qNear})

                # add the line between qNear and qNew to the plot
                self.update_plots(x, y, line_segments,
                                  lines, plot, fig, qNew, qNear)

            goalCollision = self.checkCollision(
                qGoal, qNew, obstacles, self.mode
            )
            if not goalCollision:

                self.G.update({qGoal: qNew})

                # add the line between qNew and qGoal to the plot
                self.update_plots(x, y, line_segments,
                                  lines, plot, fig, qGoal, qNew)

                self.draw_goal_path(qGoal, ax, fig)

                return self.G

            if i == 5:
                time.sleep(20)

            plt.pause(0.001)


def initialize_arg_parse():
    parser = argparse.ArgumentParser(
        description='Process arguements to decide the type of obstacle the RRT\
            algorithm will have to search around, and in some cases the number\
                of obstacles.')
    parser.add_argument('-m', '--mode', type=int, metavar='',
                        default=0, help='an integer to define the mode')
    parser.add_argument('-o', '--obstacles', type=int, metavar='',
                        default=20, help='an integer to define the number of obstacles')

    return parser


def main():

    parser = initialize_arg_parse()
    args = parser.parse_args()

    rrt = RRT(args.mode, args.obstacles)

    try:
        rrt.rrt_algo()
    except KeyboardInterrupt:
        exit()

    plt.ioff()  # turn off interactive mode.
    plt.show()


if __name__ == "__main__":
    main()
