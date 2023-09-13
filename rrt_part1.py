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
qInit = (D[0]/2,D[1]/2)
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

    xdir = qRand[0] - qNear[0]
    ydir = qRand[1] - qNear[1]

    theta = np.arctan2(ydir,xdir)
    xDiff = np.cos(theta) * delta
    yDiff = np.sin(theta) * delta
    help = np.sin(theta)

    qNew = (qNear[0] + xDiff, qNear[1] + yDiff)

    return qNew

def rrt_algo(qInit, K, delta, D):

    x, y, lines = [],[],[]

    fig, ax, plot, line_segments = draw_plots()

    G_temp = {qInit: []}
    G.update(G_temp)
    for i in range(K):
        qRand = RANDOM_CONFIGURATION(D)
        qNear = NEAREST_VERTEX(qRand,G)
        qNew = NEW_CONFIGURATION(qNear,qRand,delta)
        G_temp = {qNew: []}
        G.update(G_temp)

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

def main():

    G = rrt_algo(qInit, K, delta, D)

    plt.ioff() #turn off interactive mode.
    plt.show() # this I don't understand. I might get rid of it, will test.

    # add some kind of dynamic element??

if __name__ == "__main__":
    main()