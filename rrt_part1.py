#!/usr/bin/python
from matplotlib.collections import LineCollection
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
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
delta = 3 # this is the incremental distance
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

    distance = D[0]*2
    # print(f"distance: {distance}")
    qRand_temp = np.array(qRand)

    qNear = ()
    for k in G:
        distance_temp = np.linalg.norm(np.asarray(qRand) - np.asarray(k))
        # print(f"distance_temp: {distance_temp}")
        if distance_temp < distance:
            distance = distance_temp
            qNear = k
    # print(G)
    # print(f"qNear:{qNear}")
    return qNear

def NEW_CONFIGURATION(qNear, qRand, delta):
    # generates a new configuraition in the tree by moving some distance, delta, from one vertex configuration towards another configuration

    vector = np.subtract(qRand, qNear)

    unitVector = vector/np.linalg.norm(vector)

    qNew = tuple(np.add(qNear, np.multiply(unitVector, delta)))

    # xdir = qRand[0] - qNear[0]
    # ydir = qRand[1] - qNear[1]

    # theta = np.arctan2(ydir,xdir)
    # xDiff = np.cos(theta) * delta
    # yDiff = np.sin(theta) * delta

    # qNew = (qNear[0] + xDiff, qNear[1] + yDiff)

    return qNew

def draw_plots(qGoal):
    # this function creates the scatter plot, line collection, and defines some of
    # their attributes.

    plt.ion() # turn on interactive mode
    fig, ax = plt.subplots() # create the sub plot

    plot = ax.scatter([],[], c='blue',s=1) # create the scatter plot
    goal = ax.scatter([qGoal[0]],[qGoal[1]], c='red', s=3)
    # goal.set_offsets(np.column_stack([,y])) # add the new x and y coordiantes to the plot
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
    plt.pause(0.00001) # pause momentarily so the plot doesn't freeze ups

def randomStart():
    qInit = []
    rng = np.random.default_rng()
    qInit.append(rng.random() * D[0])
    qInit.append(rng.random() * D[1])
    return tuple(qInit)

def randomObstacles(ax, qInit, qGoal):
    # add some circles
    rng = np.random.default_rng() # add the random number generator

    numObstacles = 20
    circles = {}
    obstacles = {}
    for i in range(numObstacles):
        successful = False

        while not successful:

            r = rng.integers(0,20)
            x = rng.integers(20, D[0] - 20)
            y = rng.integers(20, D[1] - 20)

            center = (x,y)
            startDist = np.linalg.norm(np.subtract(center,qInit))
            goalDist = np.linalg.norm(np.subtract(center,qGoal))

            if startDist > r and goalDist > r:
                circles[f"circles{i}"] = Circle((x,y),r)
                ax.add_patch(circles[f"circles{i}"])
                obstacles[f"o{i}"] = [x,y,r]
                successful = True
            # else:
                # print("invalid cirle!")
        

    return obstacles

def randomGoal():
    rng = np.random.default_rng()

    qGoal = []
    qGoal.append(rng.integers(20, D[0] - 20))
    qGoal.append(rng.integers(20, D[1] - 20))

    return qGoal

def checkCollision(qNew, qNear, obstacles):
    # this function will check to see if the line segment resulting from the newest point
    # collides with the walls of any of the obstacles.
    qLine = np.subtract(qNew, qNear) # create a numpy array representing a vector between the new node and the node closest to it.

    unitQ = np.linalg.norm(qLine)

    for key in obstacles:
        x,y,r = obstacles[key] # retrieve the center point and radius for the obstacle
        oLine = np.subtract((x,y),qNear) # create a numpy array representing a vector between the center of the circle and the new node
        # unitO = np.linalg.norm(oLine)

        dist = np.linalg.norm(np.cross(qLine,oLine))/unitQ
        # print(f"dist: {dist}, r: {r}, dist < r: {dist < r}")/

        if dist < r and np.dot(qLine, oLine) > 0:
            return True # if a collision is found, return true
        
    return False # if no collisions are found, return false
    
def rrt_algo(qInit, qGoal, K, delta, D):

    x, y, lines = [],[],[]

    fig, ax, plot, line_segments = draw_plots(qGoal)
    obstacles = randomObstacles(ax, qInit, qGoal)
    
    G.update({qInit: []})
    for i in range(K):
        successful = False
        while not successful:
            qRand = RANDOM_CONFIGURATION(D)
            qNear = NEAREST_VERTEX(qRand,G)
            qNew = NEW_CONFIGURATION(qNear,qRand,delta)
            # print("qNew:")
            collision = checkCollision(qNew, qNear, obstacles)
            if not collision:
                G[qNear].append(qNew)
                G.update({qNew: []})
                successful = True
                # these variables are needed for updating the plot
                update_plots(x,y,line_segments,lines, plot, fig, qNew, qNear)

            # print("qGoal: ")
            qNear = NEAREST_VERTEX(qGoal,G)
            goalCollision = checkCollision(qGoal, qNear, obstacles)
            if not goalCollision:
                update_plots(x,y,line_segments,lines,plot,fig,qGoal,qNear)
                return G
    return G

def main():
    qInit = randomStart()
    qGoal = randomGoal()
    G = rrt_algo(qInit, qGoal, K, delta, D)
    # print(f"G: {G}")
    plt.ioff() #turn off interactive mode.
    plt.show()

    # add some kind of dynamic element??

if __name__ == "__main__":
    main()