#!/usr/bin/python
from matplotlib.collections import LineCollection
import matplotlib.pyplot as plt
import numpy as np
import imageio.v3 as im


# hash map and index each node
# linked lists?
# binary tree
# starting point, choose random point in space, find node in tree that's closest to the point, look at the direction between the node and the pint, move forwar set disgtanc e and add another node
# the random point that you pick is not the node that you add to the tree

# im going to use a graph. each key in the graph will be a node, and the value for each key will be its child nodes. This way it will be very easy for me to walk up and down the nodes.
# each of the keys and values should be variables that are equal to tuples. the tuple is a position.

# what about [key: ((posx, posy), child, child)] and then the key can be a hashmap

G = {} # this is the graph that will hold all the nodes
delta = 5 # this is the incremental distance
D = (200,200) # this is the domain
# qInit = [D[0]/2,D[1]/2]
K = 1000

def RANDOM_CONFIGURATION(D):
    # Generates a random position in the domain (D).

    rng = np.random.default_rng()
    
    x = rng.random()*D[0]
    y = rng.random()*D[1]

    qRand = (x,y) 
    # print(f"qRand: {qRand}")

    return qRand

def NEAREST_VERTEX(qRand, G): 
    # finds the vertex in that is closest to the given position using the Euclidean metric.

    distance = max(D)

    qRand_temp = np.array(qRand)

    qNear = tuple()
    for k in G:
        k_temp = np.array(k)
        distance_temp = np.linalg.norm(k_temp - qRand_temp)
        if distance_temp < distance:
            distance = distance_temp
            qNear = k
    return qNear

def NEW_CONFIGURATION(qNear, qRand, delta):
    # generates a new configuraition in the tree by moving some distance, delta, from one vertex configuration towards another configuration

    xdir = qRand[0] - qNear[0]
    ydir = qRand[1] - qNear[1]

    theta = np.arctan2(ydir,xdir)
    xDiff = np.cos(theta) * delta
    yDiff = np.sin(theta) * delta

    qNew = (qNear[0] + xDiff, qNear[1] + yDiff)

    return qNew

def checkCollision(qNew, qNear, obstacles):
    # this function will check to see if the line segment resulting from the newest point
    # collides with the walls of any of the obstacles.
    qLine = createVectorFromTwoPoints(qNew, qNear) # returns a numpy array

    for key in obstacles:
        x,y,r = obstacles[key]
        oLine = createVectorFromTwoPoints((x,y),qNew)


def createVectorFromTwoPoints(point1, point2):
    vector = []
    x1,y1 = point1
    x2,y2 = point2
    
    vector[0] = x1-x2
    vector[1] = y1-y2

    vector = np.array(vector)

    return vector

def rrt_algo(qInit, qGoal, K, delta, D):

    x, y, lines = [],[],[]

    fig, ax, plot, line_segments = draw_plots()
    obstacles, ax = randomObstacles(ax, qInit, qGoal)


    
    G.update({qInit: []})
    for i in range(K):
        successful = False
        while not successful:
            qRand = RANDOM_CONFIGURATION(D)
            qNear = NEAREST_VERTEX(qRand,G)
            qNew = NEW_CONFIGURATION(qNear,qRand,delta)
            if not checkCollision(qNew, qNear, obstacles):
                successful = True
                G.update({qNew: []})

                # these variables are needed for updating the plot
                x, y, line_segments, lines, plot, fig = update_plots(x,y,line_segments,lines, plot, fig, qNew, qNear)

    return G

def draw_plots():
    # this function creates the scatter plot, line collection, and defines some of
    # their attributes.

    plt.ion() # turn on interactive mode
    fig, ax = plt.subplots() # create the sub plot

    plot = ax.scatter([],[], c='blue',s=1) # create the scatter plot
    line_segments = LineCollection([], colors='blue',linestyle='solid')
    ax.add_collection(line_segments)
    ax.set_xlim(0,D[0])
    ax.set_ylim(0,D[1])

    plt.draw()

    return fig, ax, plot, line_segments
    

def update_plots(x,y,line_segments, lines, plot, fig, qNew, qNear):
    # this function is meant to be called in each iteration of the for lloop inside
    # rrf_algo(). It updates the plot with new nodes as they're generated.

    lines.append([qNear,qNew]) # adding the new point, and the point it was closest to, to the lines list so that a line can be drawn on the plot
    line_segments.set_segments(lines) # adding the new line to the plot

    x.append(qNew[0]) # add the x coordinate of the new point to the x coordinate list
    y.append(qNew[1]) # add the y coordinate of the new point to the y coordinate list
    plot.set_offsets(np.column_stack([x,y])) # add the new x and y coordiantes to the plot

    fig.canvas.draw_idle() # update the canvas
    plt.pause(0.00001) # pause momentarily so the plot doesn't freeze up

    return x, y, line_segments, lines, plot, fig

def randomStart():
    qInit = []
    rng = np.random.default_rng()
    qInit.append(rng.random() * D[0])
    qInit.append(rng.random() * D[1])
    return tuple(qInit)

def randomObstacles(ax, qInit, qGoal):
    # add some circles
    rng = np.random.default_rng() # add the random number generator

    numObstacles = 5
    circles = {}
    obstacles = {}
    for i in range(numObstacles):
        successful = False

        while not successful:

            r = rng.integers(0,20)
            x = rng.integers(20, D[0] - 20)
            y = rng.integers(20, D[1] - 20)

            qInit_temp = np.array(qInit)
            qGoal_temp = np.array(qGoal)
            center = np.array(x,y)
            startDist = np.linalg.norm(center - qInit_temp)
            goalDist = np.linalg.norm(center - qGoal_temp)

            if startDist > r and goalDist > r:
                circles[f"circles{i}"] = plt.Circle((x,y),r)
                ax.add_artist(circles[f"circles{i}"])
                obstacles[f"o{i}"] = [x,y,r]
                successful = True
            else:
                print("invalid cirle!")
        

    return obstacles, ax

def randomGoal():
    rng = np.random.default_rng()

    qGoal = []
    qGoal.append(rng.integers(20, D[0] - 20))
    qGoal.append(rng.integers(20, D[1] - 20))

    return qGoal
    

def main():
    qInit = randomStart()
    qGoal = randomGoal()
    G = rrt_algo(qInit, qGoal, K, delta, D)

    plt.ioff() #turn off interactive mode.
    plt.show() # this I don't understand. I might get rid of it, will test.

    # add some kind of dynamic element??

if __name__ == "__main__":
    main()