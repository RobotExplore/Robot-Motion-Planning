// A toy example of Hybrid A* Algorithm for self-driving car.
// By Haowei Zhao on 02/08/2019
// Based on Udacity Self Driving Car Nanodegree lecture on path planning.



#include <iostream>
#include <vector>
#include "hybrid_astar_search.h"

const int _ = 0;
const int X = 1;

/**
 * You can modify the map below
 */

std::vector<std::vector<int>> GRID = {
        {_,X,X,_,_,_,_,_,_,_,X,X,_,_,_,_,},
        {_,X,X,_,_,_,_,_,_,X,X,_,_,_,_,_,},
        {_,X,X,_,_,_,_,_,X,X,_,_,_,_,_,_,},
        {_,X,X,_,_,_,_,X,X,_,_,_,X,X,X,_,},
        {_,X,X,_,_,_,X,X,_,_,_,X,X,X,_,_,},
        {_,X,X,_,_,X,X,_,_,_,X,X,X,_,_,_,},
        {_,X,X,_,X,X,_,_,_,X,X,X,_,_,_,_,},
        {_,X,X,X,X,_,_,_,X,X,X,_,_,_,_,_,},
        {_,X,X,X,_,_,_,X,X,X,_,_,_,_,_,_,},
        {_,X,X,_,_,_,X,X,X,_,_,X,X,X,X,X,},
        {_,X,_,_,_,X,X,X,_,_,X,X,X,X,X,X,},
        {_,_,_,_,X,X,X,_,_,X,X,X,X,X,X,X,},
        {_,_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,},
        {_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,X,},
        {_,X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,},
        {X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,}};


int main() {

    HybridAStar HBASTAR; // Hybrid A* object

    // Set the goal below
    HybridAStar::State goal;
    goal.x = (double)GRID[0].size()-1;
    goal.y = (double)GRID.size()-1;
    goal.theta = 0.0; // Does not matter in this problem setup
    goal.f = 999999; // Initially a very large number
    goal.g = 999999; // Initially a very large number

    // Set the start below
    HybridAStar::State start;
    start.g = 0;
    start.theta = 0.0;
    start.x = 0.0;
    start.y = 0.0;
    start.f = start.g+HBASTAR.heuristic(start,goal);

    // Output the map in terminal
    std::cout<< "Finding path through the map"<<std::endl;
    for (int i =0; i < GRID.size(); ++i){
        for (int j = 0; j < GRID[0].size(); ++j) {
            std::cout<<GRID[i][j]<<",";
        }
        std::cout<<std::endl;
    }

    HybridAStar::Path path = HBASTAR.search(start,goal,GRID);

    // Print path in the terminal
    std::vector<HybridAStar::State> pathConstructed = HBASTAR.reconstructPath(path.parent,start,path.final);
    for (int k = pathConstructed.size()-1; k >= 0 ; --k) {
        std::cout<<"**********************"<<std::endl;
        std::cout<<"Step: "<< pathConstructed[k].g<<std::endl;
        std::cout<<"x: "<<pathConstructed[k].x<<", y: "<<pathConstructed[k].y<<std::endl;
        std::cout<<"theta: "<< pathConstructed[k].theta<<std::endl;
    }
    return 0;
}