# -*- coding: utf-8 -*-
"""
AI Project one: Maze Generation and Solving
"""

import numpy as np
from matplotlib import pyplot
import random
# Point Class code here:
class Point:
    def __init__(self,xValue,yValue):
        self.xValue = xValue
        self.yValue = yValue
    def __getitem__(self,arg):
        point=([self.xValue,self.yValue])
        return (point[arg])
    def __str__(self):
        return str((str(self.xValue),str(self.yValue)))
    def getCoord(self):
        return (self.xValue, self.yValue)
import heapq

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]
def cost(from_node, to_node):
    if (from_node[0] == to_node[0]) or (from_node[1] == to_node[1]):
        cost = 1
    else:
        cost = 2
    return cost
def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)
def neighbors(graph,current):
    rList=[]
    for x in range (current[0]-1,current[0]+2):
        for y in range (current[1]-1,current[1]+2):
            if graph[x,y]==0:
                if x>0 and y>0:
                    rList.append(Point(x,y))
    return rList
def aStarActor(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        print(current)
        if current == goal:
            break
        for next in neighbors(graph,current):
            new_cost = cost_so_far[current] + cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far


Dim = 50
Nstp= 100

maze= np.zeros((Dim, Dim))
xx= np.zeros((Dim, Dim))




            
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
    
for x in range (49):
    for y in range (49):
        if maze[x+1,y+1]==1 or maze[x+1,y-1]==1 or maze[x-1,y+1]==1 or maze[x-1,y-1]==1:
            if random.randint(1,4)<2:
                maze[x,y]=0
                
for x in range (50):
    for y in range (50):
        if (x==0) or (y==0) or (x==49) or (y==49):
            maze[x,y]= 1
for x in range (3):
    for y in range (3):
            maze[x,y]= 0
for x in range (47, 50):
    for y in range (47,50):
            maze[x,y]= 0
img = pyplot.imshow(maze,interpolation='nearest')
#==============================================================================
# aStarActor(maze, (0,0), (49,49))
#==============================================================================
pyplot.show()
