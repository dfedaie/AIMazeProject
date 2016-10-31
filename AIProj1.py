# -*- coding: utf-8 -*-
"""
AI Project one: Maze Generation and Solving
By Ben, Deana, and Mariah
"""

import numpy as np #necessary for creating the maze
from matplotlib import pyplot #necessary for displaying the maze
import copy #necessary because I hate how pointers work in python
import random #necessary to randomize the maze
# Point Class code here:
class Point: #custom point class that stores two values, and can be handled beautifully and printed beautifully. Beautiful points. 
    def __init__(self,xValue,yValue):
        self.xValue = xValue
        self.yValue = yValue
    def __getitem__(self,arg):
        point=([self.xValue,self.yValue])
        return (point[arg])
    def __repr__(self):
        return str((str(self.xValue),str(self.yValue)))
    def __str__(self):
        return str((str(self.xValue),str(self.yValue)))
    def getCoord(self):
        return (self.xValue, self.yValue)
import heapq

class PriorityQueue: #Priority Queue code used found at http://www.redblobgames.com/pathfinding/a-star/introduction.html
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
def heuristic(a, b): #finds the distance between two points. used to find H costs
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)
    
def neighbors(graph,current): #returns a list of empty points adjacent to the current square.
    rList=[] 
    for x in range (current[0]-1,current[0]+2):
        for y in range (current[1]-1,current[1]+2):
            if (graph[x,y]==0) and (x>0 and y>0):
                add= Point(x,y)
                rList.append(add.getCoord())
    return rList


def aStarActor(graph):#A* algorithm
    #defines the start and end points
    start=(0,0)
    goal=(48,48)
    
    #creates the frontier queue, and puts the starting location into it.
    frontier = PriorityQueue()
    frontier.put(start, 0)
    
    #initilize variables
    came_from = {} #each location, listed by the location that preceded it. 
    cost_so_far = {} #associated costs of each move, listed by the point moved to.
    came_from[start] = None #didn't come from nowhere
    cost_so_far[start] = 0 #didn't cost nothing. 
    
    while (not frontier.empty()): #as long as the frontier has additional squares to expand to, continue.
        current = frontier.get() #returns the highest priority space in the frontier
        if current == goal: #if the frontier covers the goal space, we're there. We did it. 
            break
        for next in neighbors(graph,current): #Shunt to neighbor code. find all movable squares from the currently observed location
            #finds the cost of the potential next move by adding the cost of the 
            #last square (and all preceding) to the projected cost of the next move
            new_cost = cost_so_far[current] + cost(current, next) 
            if next not in cost_so_far or new_cost < cost_so_far[next]: # if the potential next move is not in cost_so_far 
                cost_so_far[next] = new_cost #add the move to the cost list
                priority = new_cost + heuristic(goal, next) #the priority of the move is equal to its move + heuristic costs
                frontier.put(next, priority) #put the next square movable into the frontier, with a priority equal to its costs
                came_from[next] = current #add the current possibility to the came_from list, tagged to the observed location.
    xx=(48,48) #starting at the goal location, change the color of all moves leading from goal to start, denoting a solved grid.
    while not(xx==(0,0)):
        xx=came_from[xx]
        graph[xx[0],xx[1]]=3
    #save the solved graph to the project folder as a png.
    pyplot.imshow(graph)
    pyplot.savefig("aStarSolution.png")
    return graph, cost_so_far[goal] #returns a solved graph, and the total cost.
    
def heuristicActor(graph): #A star variant which ignores f costs
    #defines the start and end points
    start=(0,0)
    goal=(48,48)
    
    #creates the frontier queue, and puts the starting location into it.
    frontier = PriorityQueue()
    frontier.put(start, 0)
    
    #initilize variables
    came_from = {} #each location, listed by the location that preceded it. 
    cost_so_far = {} #associated costs of each move, listed by the point moved to.
    came_from[start] = None #didn't come from nowhere
    cost_so_far[start] = 0 #didn't cost nothing. 
    
    while (not frontier.empty()): #as long as the frontier has additional squares to expand to, continue.
        current = frontier.get() #returns the highest priority space in the frontier
        if current == goal: #if the frontier covers the goal space, we're there. We did it. 
            break
        for next in neighbors(graph,current): #Shunt to neighbor code. find all movable squares from the currently observed location
            #finds the cost of the potential next move by adding the cost of the 
            #last square (and all preceding) to the projected cost of the next move
            new_cost = cost_so_far[current] + cost(current, next) 
            if next not in cost_so_far or new_cost < cost_so_far[next]: # if the potential next move is not in cost_so_far 
                cost_so_far[next] = new_cost #add the move to the cost list
                priority = heuristic(goal, next) #the priority of the move is equal to its heuristic cost.
                frontier.put(next, priority) #put the next square movable into the frontier, with a priority equal to its costs
                came_from[next] = current #add the current possibility to the came_from list, tagged to the observed location.
    xx=(48,48) #starting at the goal location, change the color of all moves leading from goal to start, denoting a solved grid.
    while not(xx==(0,0)):
        xx=came_from[xx]
        graph[xx[0],xx[1]]=3
    #save the solved graph to the project folder as a png.
    pyplot.imshow(graph)
    pyplot.savefig("heuristicSolution.png")
    return graph, cost_so_far[goal] #returns a solved graph, and the total cost.
    
def fCostActor(graph): #A star variant which ignores h costs
    #defines the start and end points
    start=(0,0)
    goal=(48,48)
    
    #creates the frontier queue, and puts the starting location into it.
    frontier = PriorityQueue()
    frontier.put(start, 0)
    
    #initilize variables
    came_from = {} #each location, listed by the location that preceded it. 
    cost_so_far = {} #associated costs of each move, listed by the point moved to.
    came_from[start] = None #didn't come from nowhere
    cost_so_far[start] = 0 #didn't cost nothing. 
    
    while (not frontier.empty()): #as long as the frontier has additional squares to expand to, continue.
        current = frontier.get() #returns the highest priority space in the frontier
        if current == goal: #if the frontier covers the goal space, we're there. We did it. 
            break
        for next in neighbors(graph,current): #Shunt to neighbor code. find all movable squares from the currently observed location
            #finds the cost of the potential next move by adding the cost of the 
            #last square (and all preceding) to the projected cost of the next move
            new_cost = cost_so_far[current] + cost(current, next) 
            if next not in cost_so_far or new_cost < cost_so_far[next]: # if the potential next move is not in cost_so_far 
                cost_so_far[next] = new_cost #add the move to the cost list
                priority =  new_cost #the priority of the move is equal to its move cost.
                frontier.put(next, priority) #put the next square movable into the frontier, with a priority equal to its costs
                came_from[next] = current #add the current possibility to the came_from list, tagged to the observed location.
    xx=(48,48) #starting at the goal location, change the color of all moves leading from goal to start, denoting a solved grid.
    while not(xx==(0,0)):
        xx=came_from[xx]
        graph[xx[0],xx[1]]=3
    #save the solved graph to the project folder as a png.
    pyplot.imshow(graph)
    pyplot.savefig("fCostSolution.png")
    return graph, cost_so_far[goal] #returns a solved graph, and the total cost.
    
    
# 7 0 1    
# 6 ☺ 2 The direction square!
# 5 4 3 Whatever number "compass" is set to, corrosponds to this square.
def go(direction,point): #given a place, and a direction, returns the square one step forwards in that direction. Used in WallHugger
    if (direction == 7) or (direction == 0) or (direction == 1): #if we move up
        xVar= -1
    elif (direction == 2) or (direction == 6): #if we move left or right
        xVar = 0
    else: #if we move down
        xVar = 1
    if (direction == 5) or (direction == 6) or (direction == 7): #if we move left
        yVar= -1
    elif (direction == 4) or (direction == 0): #if we move up or down
        yVar = 0
    else: #if we move right
        yVar = 1
    moveTo=Point(point[0]+xVar,point[1]+yVar)
    return moveTo
    
#navigates the maze by hugging one wall the entire time.
def wallHugger(graph):
    #initialization
    current = Point(0,0)
    goal = Point(48,48)
    compass = 0
    toteCost=0
    #while we haven't made it to the end, keep truckin. unsolvable mazes throw up errors during the other actors turns, so I don't need to
    #deal with that here.
    while not (current.getCoord() == goal.getCoord()):
        #defines a name for each of squares to the left of the current space.
        backLeft = go((compass-3)%8, current)           
        backLeftSquare = graph[backLeft[0], backLeft[1]]
        left = go((compass-2)%8, current)
        leftSquare = graph[left[0], left[1]]
        frontLeft = go((compass-1)%8, current)
        frontLeftSquare = graph[frontLeft[0], frontLeft[1]]
        
#all the following statements have the conditions that a square that can be moved to is empty and within the grid        
        #if the square back and to the left of the current is open, you must move turn towards it and there, to continue hugging the wall
        if ((backLeftSquare == 0) or (backLeftSquare == 3)) and ((backLeft[0]> 0) and (backLeft[0]<50) and (backLeft[1]> 0) and (backLeft[1]<50)):
                compass = (compass-3)%8 #turn towards where youre moving
                graph[current[0],current[1]]=3 #mark the spot you're on, to show you've been there
                toteCost+=cost(current,backleft) #add the cost of the move to the total cost
                current=backLeft #take a step forwards
        #if the backleft is nonmoveable, and the directly left square is open, thats where you gotta go.
        elif ((leftSquare == 0) or (leftSquare == 3)) and ((left[0]> 0) and (left[0]<50) and (left[1]> 0) and (left[1]<50)):
                compass = (compass-2)%8#turn towards where youre moving
                graph[current[0],current[1]]=3 #mark the spot you're on, to show you've been there
                toteCost+=cost(current,left) #add the cost of the move to the total cost
                current=left #take a step forwards        
        #then the front left square
        elif ((frontLeftSquare == 0) or (frontLeftSquare == 3)) and ((frontLeft[0]> 0) and (frontLeft[0]<50) and (frontLeft[1]> 0) and (frontLeft[1]<50)):
                compass = (compass-1)%8#turn towards where youre moving
                graph[current[0],current[1]]=3 #mark the spot you're on, to show you've been there
                toteCost+=cost(current,frontLeft)  #add the cost of the move to the total cost               
                current=frontLeft  #take a step forwards
        #only once you've felt up the wall to the left can you keep moving forwards.
        elif ((graph[go(compass, current)[0], go(compass, current)[1]] == 0) or (graph[go(compass, current)[0], go(compass, current)[1]] == 3)) and ((go(compass, current)[0]> 0) and (go(compass, current)[0]<50) and (go(compass, current)[1]> 0) and (go(compass, current)[1]<50)):
                graph[current[0],current[1]]=3 #mark the spot you're on, to show you've been there
                toteCost+=cost(current,go(compass, current))   #add the cost of the move to the total cost  
                current=go(compass, current) #take a step forwards
        #if you're boxed in on the left, you may turn right.
        else:
            compass = (compass+1)%8 #turn 45 degrees. check the direction square for more details
    #saves the completed maze as a png in the project folder
    pyplot.imshow(graph)
    pyplot.savefig("wallHugSolution.png")
    return graph, toteCost #return the complete maze, and the total cost

    
#maze initialized as a 50 X 50 array of zeros. 
Dim = 50
Nstp= 100
maze= np.zeros((Dim, Dim))
xx= np.zeros((Dim, Dim)) 




#initial state of maze is random            
for x in range (0,50):
    for y in range (0,50):
        num=random.random()
        if num>.6:
            maze[x,y]=1
#uses cellular automata to generate a maze looking structure
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
    

#seals the border of the maze               
for x in range (50):
    for y in range (50):
        if (x==0) or (y==0) or (x==49) or (y==49):
            maze[x,y]= 1
            
#creates an empty, "safe zone" around the goal, to ensure the goal is reachable.
for x in range (3):
    for y in range (3):
            maze[x,y]= 0
for x in range (46, 49):
    for y in range (46,49):
            maze[x,y]= 0

#uses copy function to give each actor a blank maze of their own.
maze1 = copy.copy(maze)
maze2 = copy.copy(maze)
maze3 = copy.copy(maze)
maze4 = copy.copy(maze)

#run all four actors, and print their costs. 
graph, starCost = aStarActor(maze1)
graph1, hcost = heuristicActor(maze2)
graph2, fcost = fCostActor(maze3)
graph3, wallcost = wallHugger(maze4)
print "Star Cost:",starCost
print "h Cost:", hcost
print "f Cost:", fcost
print "Wall Cost:", wallcost
#display the base maze.
img = pyplot.imshow(maze,interpolation='nearest')
pyplot.show()

