#!/usr/bin/python

import matplotlib as plot
import numpy as np
import imageio.v3 as im
import time

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
D = np.array([100,100]) # this is the domain
qInit = np.array([50,50])

def RANDOM_CONFIGURATION(D):
    rng = np.random.Generator(seed=60)
    
    x = rng.integers(low=0,high=D[0]+1)
    y = rng.integers(low=0,high=D[1]+1)

    qRand = np.array([x,y])

    return qRand

def NEAREST_VERTEX(qRand, G): 
    distance = 0
    qNear = ()
    for k in G:
        distance_temp = np.linalg.norm(k - qRand)
        if distance_temp > distance:
            distance = distance_temp
            qNear = k
    return qNear

def NEW_CONFIGURATION(qNear, qRand, delta):

    slope = calc_slope(qNear, qRand)

    theta = np.tan(slope)
    xDiff = np.cos(theta)
    yDiff = np.sin(theta)

    qNew = np.array([xDiff,yDiff])

    return qNew



def rrt_algo(qInit, K, delta, D):
    G[qInit] = []
    for i in range(K):
        qRand = RANDOM_CONFIGURATION(D)
        qNear = NEAREST_VERTEX(qRand,G)
        qNew = NEW_CONFIGURATION(qNear,qRand,delta)
        G[qNear] = G[qNear].append(qNew)
        G[qNew] = []

def calc_slope(point1, point2):
    x1, y1 = point1
    x2, y2 = point2

    slope = (y1 - y2) / (x1 - x2)

    return slope

def main():
    print("hi")

if "__name__" == "__main__":
    main()