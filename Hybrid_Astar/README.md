### Hybrid A* Algorithm
Hybrid A* Algorithm was initially implemented by Stanford team in DARPA Urben Challange to plan path in the parking lot for their self-driving car, Junior. The algorithm is descirbed in this paper: [Junior: The Stanford Entry in the Urban Challenge](http://robots.stanford.edu/papers/junior08.pdf)  

In this setup, I use a 3D configuration space, which includes x,y and theta. In a real world setup, there should be an additional dimension, describing whether the car is moving forward or backward.

In the following pseudocode,`SPEED` is the speed of the car , `LENGTH` is the length of the car.  

* Pseudocode:
```
def Expend(current, goal):
	for dTheta in range(minTheta,maxTheta,step):
	
		nextStateList = [];
		
		// Use bicycle motion model to update state
		omega = SPEED/LENGTH *tan(current.theta);
		nextTheta = current.theta+omega;
		nextX = current.x + SPEED*cos(current.theta); // Or sin(theta), depends on the coordinate setup
		nextY = current.y + SPEED*sin(current.theta); // Or cos(theta), depends on the coordinate setup
		nextG = current.g+1;
		nextF = nextG + Heuristic(nextState,goal);
		
		Put nextState into the nextStateList;
		
	return nextStateList;
```
```
def HybridAStar(start, goal, map):
	openList = [];
	
	// A closed list to save the cells that have been 
	// explored
	closedList = [];
	
	// A parent list to save the predecessors of current node
	parentLIst = [];
	
	Put the start node into the openlist;
	Put the start node into the closedList;
	
	while The openList is not empty:
		Sort the open list using f value as the key;
		current = openList[0];
		Pop openlist[0];
		
		if current == goal:
			return parentList;
		
		nextStatesList = Expend(current, goal);
		
		for state in nextStatesList:
			if state is in map:
				continue;
			if state is not in closedList and state is not a obstacle:
				Put state into openList;
				Put current into parentList;
				Put state into the closedList;
	
	Report path is not found;
	return parentList;
	
```
### Possible improvement:
We can use dynamic programming to calculate the heuristic. This would make the algorithm explore less gird cells therefore leads to better performance.
#
Some code is based on the lecture material of Udacity Self-Driving Car Nanodegree. 
