//
// Created by Haowei Zhao on 2/5/19.
// Based on Udacity Self Driving Car Nanodegree lecture on path planning.
//

#include "hybrid_astar_search.h"
#include <math.h>
#include <algorithm>
#include <iostream>

// Sorting function
bool compFunction(HybridAStar::State &state1, HybridAStar::State &state2){
    return state1.f<state2.f;
}


HybridAStar::HybridAStar()=default;

HybridAStar::~HybridAStar()=default;


// Here we are trying to find a mapping from theta(in radius) to the theta space in 3D configuration space.
// The function would convert theta to the "stack" in 3D C space, which contains different x and y configurations
unsigned long HybridAStar::thetaToStack(double thetaIn) {
    double thetaNew = fmod((thetaIn+2*M_PI),(2*M_PI));
    unsigned long thetaIndex = (int)(round(thetaNew*NUM_THETA_CELLS/(2*M_PI)))%NUM_THETA_CELLS;
    return thetaIndex;
}

// Here we are trying to find a mapping from x and y position to index to locate the position in configuration space.
int HybridAStar::convertToIndex(double positionIn) {
    int positionIndex = (int)floor(positionIn);
    return positionIndex;
}


// The function to calculate heuristic value, which is Euclidean distance in this case.
// TODO: Dynamic programming can be used to assign cost function to each grid cell,which can be used as heuristic value. This leads to better efficiency
double HybridAStar::heuristic(const HybridAStar::State& current,
        const HybridAStar::State& goal) {
    return sqrt(pow(current.x-goal.x,2)+pow(current.y-goal.y,2));
}

// Expand function of A* function
std::vector<HybridAStar::State> HybridAStar::expand(const HybridAStar::State &currentState,
        const HybridAStar::State &goalState) {

    double x = currentState.x;
    double y = currentState.y;
    double theta = currentState.theta;
    int g = currentState.g;

    std::vector<HybridAStar::State> nextStateList;
    g +=1;
    for (double i = -35; i<40; i+=5){

        double theta2 = theta+(speed/lengthOfCar)*tan(i*M_PI/180.0);
        if (theta2<0) theta2 +=2*M_PI; // We always assume theta is not less then 0
        // Update x and y position using bicycle motion model below.
        double x2 = x+speed*sin(theta);
        double y2 = y+speed*cos(theta);


        HybridAStar::State nextState;
        nextState.x = x2;
        nextState.y = y2;
        double f = g+heuristic(nextState,goalState); // Update new f value
        nextStateList.push_back(State{g,x2,y2,f,theta2});
    }

    return nextStateList;
}


// Hybrid A* search algorithm
HybridAStar::Path HybridAStar::search(const HybridAStar::State &start, const HybridAStar::State &goal,
        const std::vector<std::vector<int>> &grid){
    unsigned long numberOFColumn = grid[0].size(); // X coordinates
    unsigned long numberOfRow = grid.size(); // Y coordinates
    std::vector<HybridAStar::State> openList;
    std::vector<std::vector<std::vector<int>>> closedList(HybridAStar::NUM_THETA_CELLS,
            std::vector<std::vector<int>>(numberOFColumn,std::vector<int>(numberOfRow))); // 3D vector for closed cell
    std::vector<std::vector<std::vector<HybridAStar::State>>> parentList(HybridAStar::NUM_THETA_CELLS,
            std::vector<std::vector<HybridAStar::State>>(numberOFColumn,std::vector<HybridAStar::State>(numberOfRow)));// 3D vector for parent cell to reconstruct the path.

    int numClosed = 1;

    // Initialization
    openList.push_back(start);
    unsigned long stack = thetaToStack(start.theta);
    parentList[stack][convertToIndex(start.y)][convertToIndex(start.x)] = start;
    closedList[stack][convertToIndex(start.y)][convertToIndex(start.x)] = 1;
    HybridAStar::State finalState = start;

    // Continuously search for next feasible grid cell when openlist is not empty
    while(!openList.empty()){
        sort(openList.begin(),openList.end(), compFunction);// Sort the open list using compFunction
        HybridAStar::State current = openList[0];
        openList.erase(openList.begin());// Remove current node from the list
        finalState = current;

        if (convertToIndex(current.x) == goal.x && convertToIndex(current.y) == goal.y){
            std::cout<< "Solution Found!"<<std::endl;
            std::cout<< "Solution found after "<< numClosed<< "grids explored"<<std::endl;
            return HybridAStar::Path{parentList,finalState};
        }

        std::vector<HybridAStar::State> nextStates = expand(current,goal);

        for (int i = 0; i<nextStates.size(); ++i){

            // Check if the node is still with in the grid
            if ((nextStates[i].x < 0 || nextStates[i].x >= grid[0].size())||
            (nextStates[i].y<0 || nextStates[i].y >= grid.size())){
                continue;
            }

            // If the node is not in the closed list and there is no obstacle, the put the node into openList and closedList.
            if (closedList[thetaToStack(nextStates[i].theta)][convertToIndex(nextStates[i].y)]
                [convertToIndex(nextStates[i].x)] ==0 &&
                grid[convertToIndex(nextStates[i].y)][convertToIndex(nextStates[i].x)] ==0){

                parentList[thetaToStack(nextStates[i].theta)][convertToIndex(nextStates[i].y)]
                [convertToIndex(nextStates[i].x)] = current;

                closedList[thetaToStack(nextStates[i].theta)][convertToIndex(nextStates[i].y)]
                [convertToIndex(nextStates[i].x)] = 1;

                openList.push_back(nextStates[i]);
                ++numClosed;
            }
        }
    }

    std::cout<<"NO VALID PATH FOUND!"<< std::endl;
    return HybridAStar::Path{parentList,finalState};
}



// Use parentList to construct the path
std::vector<HybridAStar::State> HybridAStar::reconstructPath(
        std::vector<std::vector<std::vector<HybridAStar::State>>> &parentList, HybridAStar::State &start,
        HybridAStar::State &finalState) {

    std::vector<State> path = {finalState};// Our goal should be the last node in the path.

    State previousState = parentList[thetaToStack(finalState.theta)][convertToIndex(finalState.y)][convertToIndex(finalState.x)];

    while (previousState.x != start.x || previousState.y != start.y){
        path.push_back(previousState);
        // We use position of current node to find its parent node in parentList.
        previousState = parentList[thetaToStack(previousState.theta)][convertToIndex(previousState.y)][convertToIndex(previousState.x)];
    }

    return path;
}
