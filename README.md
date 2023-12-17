Title: Rapidly-Exploring Random Tree Algorithm
Author: Graham Clifford

This project implements the rapidly-expanding random tree algorithm, first developed by Steven LaValle in 1988.

A link to his original paper is [here](https://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf).

A Rapidly-Exploring Random Tree (RRT) is a fundamental path planning algorithm in robotics.

An RRT consists of a set of vertices, which represent configurations in some domain D and edges, which connect two vertices. The algorithm randomly builds a tree in such a way that as the number of vertices n increases to \\\( \infty \\\), the vertices are uniformly distributed across the domain \\\(D \subset R^n \\\).

The algorithm, as presented below, has been simplified from the original version by assuming a robot with dynamics \\\( \dot{q} = u \\\) where \\\(\lVert u \rVert = 1 \\\) and assuming that we integrate the robot's position forward for \\\(\Delta{t} = 1 \\\).

## How to Run
To run the file, navigate to the directory where the script is stored and run:
```
$ python rrt.py
```
The file has two modes:
- **Mode 0**: The only obstacle is the Northwestern "N"
- **Mode 1**: There are many circular obstacles. The default is 20 circles.

You can also specify arguments that allow you to change modes, or determine the number of obstacles.
```
usage: rrt.py [-h] [-m] [-o]

Process arguments to decide the type of obstacle the RRT algorithm will have to search around, and
in some cases the number of obstacles.

options:
  -h, --help         show this help message and exit
  -m , --mode        an integer to define the mode
  -o , --obstacles   an integer to define the number of obstacles

```
Here's an example with the arguments:
```
$ python rrt.py -m 1 -o 40
```