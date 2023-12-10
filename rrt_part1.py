from matplotlib.collections import LineCollection
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import cv2


class RRT():
    def __init__(self):
        self.G = {}  # this is the graph that will hold all the nodes
        self.delta = 3  # this is the incremental distance
        self.D = (200, 200)  # this is the domain
        # qInit = (self.D[0]/2, D[1]/2)
        self.K = 1000

    def RANDOM_CONFIGURATION(self):
        '''
        Generate a random position in the domain (D).

        Args:
        ----
        D (int tuple): the domain space.
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
            - qRand (list): a random position in the domain.
            - G (list): the list of nodes in the domain.
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
        fig, ax = plt.subplots()  # create the sub plot

        ax.imshow

        plot = ax.scatter([], [], c="blue", s=1)  # create the scatter plot
        goal = ax.scatter([qGoal[0]], [qGoal[1]], c="red", s=3)
        # goal.set_offsets(np.column_stack([,y])) # add the new x and y
        # coordiantes to the plot
        line_segments = LineCollection([], colors="blue", linestyle="solid")
        ax.add_collection(line_segments)
        ax.set_xlim(0, self.D[0])
        ax.set_ylim(0, self.D[1])

        plt.draw()

        return fig, ax, plot, line_segments

    def update_plots(self, x, y, line_segments, lines, plot, fig, qNew, qNear):
        # this function is meant to be called in each iteration of the for loop
        # inside rrt_algo(). It updates the plot with new nodes as they're
        # generated.

        # adding the new point, and the point it was
        lines.append([qNear, qNew])
        # closest to, to the lines list so that a line can be drawn on the plot
        line_segments.set_segments(lines)  # adding the new line to the plot
        # add the x coordinate of the new point to the x coordinate list
        x.append(qNew[0])
        # add the y coordinate of the new point to the y coordinate list
        y.append(qNew[1])
        plot.set_offsets(
            np.column_stack([x, y])
        )  # add the new x and y coordiantes to the plot

        fig.canvas.draw_idle()  # update the canvas
        plt.pause(0.00001)  # pause momentarily so the plot doesn't freeze up

    def checkCollision(self, qThere, qHere, obstacles):
        '''
        Check to see if there is a collision.

        Check to see if there is a collision with an obstacle between
        qThere and qHere, both of which are points in the graph.

        Args:
        ----
        qThere: Point that is somewhere else.
        qHere: Point that is right where we're located.
        obstacles: A list of the positions of obstacles.
        '''
        qLine = np.subtract(qThere, qHere)

        qDistance = np.linalg.norm(np.asarray(qThere) - np.asarray(qHere))

        i = 0
        for key in obstacles:
            # retrieve the center point and radius for the obstacle
            x, y, r = obstacles[key]

            oDistance = np.linalg.norm(np.asarray((x, y)) - np.asarray(qHere))

            # create a numpy array representing a vector between the center of
            # the circle and the new node
            oLine = np.subtract((x, y), qHere)

            dist = np.linalg.norm(np.cross(oLine, qLine)) / qDistance

            dot_product = np.dot(qLine, oLine)
            i += 1

            if dist < r and dot_product > 0 and qDistance > oDistance - r:
                return True  # if a collision is found, return true

        return False  # if no collisions are found, return false

    def randomStart(self):

        rng = np.random.default_rng()

        x = rng.random() * self.D[0]
        y = rng.random() * self.D[1]

        qInit = (x, y)

        return qInit

    def randomGoal(self):
        rng = np.random.default_rng()

        x = rng.integers(20, self.D[0] - 20)
        y = rng.integers(20, self.D[1] - 20)

        qGoal = (x, y)

        return qGoal

    def randomObstacles(self, ax, qInit, qGoal):
        # add some circles
        rng = np.random.default_rng()  # add the random number generator

        numObstacles = 20
        circles = {}
        obstacles = {}
        for i in range(numObstacles):
            successful = False

            while not successful:
                r = rng.integers(0, 20)
                x = rng.integers(20, self.D[0] - 20)
                y = rng.integers(20, self.D[1] - 20)
        # _, binary_image = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY

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

    def load_image_binary(self, ax, qGoal):
        image = cv2.imread("N_map.png", cv2.IMREAD_COLOR)

        ax.imshow(image, origin='lower')

    def rrt_algo(self, qInit, qGoal):
        x, y, lines = [], [], []

        fig, ax, plot, line_segments = self.draw_plots(qGoal)
        obstacles = self.randomObstacles(ax, qInit, qGoal)
        self.load_image_binary(ax, qGoal)

        self.G.update({qInit: []})
        for i in range(self.K):
            successful = False
            while not successful:
                qRand = self.RANDOM_CONFIGURATION()
                qNear = self.NEAREST_VERTEX(qRand)
                qNew = self.NEW_CONFIGURATION(qNear, qRand)

                goalCollision = self.checkCollision(
                    qGoal, qNew, obstacles
                )
                if not goalCollision:

                    qReverse = qNew

                    self.G.update({qNew: [qNear]})
                    self.update_plots(x, y, line_segments,
                                      lines, plot, fig, qNew, qNear)
                    self.update_plots(x, y, line_segments,
                                      lines, plot, fig, qGoal, qNew)

                    return self.G

                collision = self.checkCollision(
                    qNew, qNear, obstacles
                )

                if not collision:

                    self.G.update({qNew: [qNear]})
                    self.update_plots(x, y, line_segments,
                                      lines, plot, fig, qNew, qNear)


def main():

    rrt = RRT()

    try:
        rrt.rrt_algo(rrt.randomStart(), rrt.randomGoal())
    except KeyboardInterrupt:
        exit()

    plt.ioff()  # turn off interactive mode.
    plt.show()

    # add some kind of dynamic element??


if __name__ == "__main__":
    main()
