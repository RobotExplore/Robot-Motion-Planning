//
// Created by Haowei Zhao on 2/5/19.
// Based on Udacity Self Driving Car Nanodegree lecture on path planning.
//

#ifndef HYBRID_ASTAR_TEST_HYBRID_BREATH_SEARCH_H
#define HYBRID_ASTAR_TEST_HYBRID_BREATH_SEARCH_H

#include <vector>

class HybridAStar{
public:

    HybridAStar();

    ~HybridAStar();


    // State of the vehicle
    struct State{
        int g;
        double x;
        double y;
        double f;
        double theta;
    };

    // Final path we are looking for
    struct Path{
        std::vector<std::vector<std::vector<State>>> parent; // Parent grid points
        State final; // Final state
    };

    std::vector<State> expand(const State &currentState, const State &goalState); // Find out next feasible grids

    unsigned long thetaToStack(double thetaIn); // Convert theta to coordinates in 3D

    int convertToIndex(double positionIn); // Convert x,y positions to index of the grid

    double heuristic(const State &current, const State &goal); // Compute the heuristic value from current to goal

    // Hybrid A* search algorithm
    Path search(const State &start, const State &goal, const std::vector<std::vector<int>> &grid);

    // Construct the path from final state to start
    std::vector<State> reconstructPath(
            std::vector<std::vector<std::vector<State>>> &parentList, State &start, State &finalState);


private:
    const unsigned long NUM_THETA_CELLS = 90;
    const double speed = 1.0; // Speed of the Car
    const double lengthOfCar = 1.0;// Length of the Car


};
#endif //HYBRID_ASTAR_TEST_HYBRID_BREATH_SEARCH_H
