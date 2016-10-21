# -*- coding: utf-8 -*-
"""
AI Project one: Maze Generation and Solving
"""
import numpy as np
from matplotlib import pyplot

Dim = 50
Nstp= 100

maze= np.zeros((Dim, Dim))
xx= np.zeros((Dim, Dim))

import random


            
for x in range (0,50):
    for y in range (0,50):
        num=random.random()
        if num>.6:
            maze[x,y]=1

for tt in range (1,Nstp):
    for ii in range (0,Dim):
        for jj in range (0,Dim):
            u=(ii-1)%Dim     
            d=(ii+1)%Dim
            lft=(jj-1)%Dim
            rt=(jj+1)%Dim
            S=maze[u,jj]+maze[d,jj]+maze[ii,lft]+maze[ii,rt]
            S=S+maze[u,lft]+maze[u,rt]+maze[d,lft]+maze[d,rt]
            if maze[ii,jj]==1:
                if S>0 and S<5:
                    xx[ii,jj]=1
                else:
                    xx[ii,jj]=0
            if maze[ii,jj]==0:
                if S==3:
                    xx[ii,jj]=1
                else:
                    xx[ii,jj]=0
    maze=xx
for x in range (50):
    for y in range (50):
        if (x==0) or (y==0) or (x==49) or (y==49):
            maze[x,y]= 1

img = pyplot.imshow(maze,interpolation='nearest')
pyplot.show()


