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
delta = 1 # this is the incremental distance
D = (100,100) # this is the domain
qInit = (50,50)
K = 1000

def RANDOM_CONFIGURATION(D):
    rng = np.random.default_rng()
    
    x = rng.random()*D[0]
    y = rng.random()*D[1]

    qRand = (x,y) 
    # print(f"qRand: {qRand}")

    return qRand

def NEAREST_VERTEX(qRand, G): 

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

    # slope = calc_slope(qRand, qNear)

    xdir = qRand[0] - qNear[0]
    ydir = qRand[1] - qNear[1]

    theta = np.arctan2(ydir,xdir)
    xDiff = np.cos(theta) * delta
    yDiff = np.sin(theta) * delta
    help = np.sin(theta)

    qNew = (qNear[0] + xDiff, qNear[1] + yDiff)

    return qNew

def rrt_algo(qInit, K, delta, D):

    plt.ion()
    fig, ax = plt.subplots()

    plot = ax.scatter([],[], c='blue', marker='o', label='Points')
    lines = LineCollection([], colors='blue', linewidths=(0.5,1,1.5,2),linestyle='solid')
    ax.set_xlim(0,D[0])
    ax.set_ylim(0,D[1])

    xValues, yValues = [],[]

    plt.draw()

    G_temp = {qInit: []}
    G.update(G_temp)
    for i in range(K):
        qRand = RANDOM_CONFIGURATION(D)
        qNear = NEAREST_VERTEX(qRand,G)
        qNew = NEW_CONFIGURATION(qNear,qRand,delta)
        G_temp = {qNew: []}
        G.update(G_temp)

    #     # plot 
        xValues.append(qNew[0])
        yValues.append(qNew[1])
        plot.set_offsets(np.column_stack([xValues,yValues]))
        fig.canvas.draw_idle()
        plt.pause(0.001)
        
    plt.waitforbuttonpress()
    return G

def calc_slope(point1, point2):
    x1, y1 = point1
    x2, y2 = point2

    slope = (y1 - y2) / (x1 - x2)

    return slope

def main():

    G = rrt_algo(qInit, K, delta, D)

    nodes = G.keys()
    xValues, yValues = zip(*nodes)

    plt.scatter(xValues,yValues,s=1)
    plt.show()


if __name__ == "__main__":
    main()