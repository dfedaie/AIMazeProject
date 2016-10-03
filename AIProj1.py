# -*- coding: utf-8 -*-
"""
AI Project one: Maze Generation and Solving
"""
import numpy as np
from matplotlib import pyplot
maze= np.zeros((50, 50))
import random

for x in range (50):
    for y in range (50):
        if (x==0) or (y==0) or (x==49) or (y==49):
            maze[x,y]= 1
        else:
            maze[x,y]= 0
            
for x in range (1,50):
    for y in range (1,50):
        num=random.random()
        if num>.6:
            maze[x,y]=1
img = pyplot.imshow(maze,interpolation='nearest')
pyplot.show()
