///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Austin Hushower and Divya Wagh
//////////////////////////////////////

#include <iostream>

// The collision checker routines
#include "CollisionChecking.h"

// Your random tree planner
#include "RTP.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include "Constants.h"
  
namespace ob = ompl::base;
namespace og = ompl::geometric;


// We are using set start and goal points for both environments rather than randomly generating them
// This function returns the specified start and goal based on the environment that is chosen (1 or 2)
// It is used by SimpleSetup later
void setStartAndGoal(ob::ScopedState<> &start, ob::ScopedState<> &goal, int env)
{
    // Environment 1
    if (env == 1) {
        std::vector<double> start_state{-8.5, 7};
        start = start_state;
        std::vector<double> goal_state{8.5, 7};
        goal = goal_state;

    // Environment 2
    } else { 
        std::vector<double> start_state{-9, 0};
        start = start_state;
        std::vector<double> goal_state{8, 0};
        goal = goal_state;
    }
}

// Planning for Point Robot
void planPoint(const std::vector<Rectangle> &obstacles, int env)
{
    // TODO: Use your implementation of RTP to plan for a point robot we construct the state space we are planning in

    // SetUp Space: x and y (R2 Space)
    auto space(std::make_shared<ob::RealVectorStateSpace>());

    // Set the bounds for the R^2 space
    space->addDimension(-SPACE_DIM, SPACE_DIM);
    space->addDimension(-SPACE_DIM, SPACE_DIM);

    // Define a simple setup class
    og::SimpleSetup ss(space);

    // Set planner
    ss.setPlanner(std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation()));
    
    // Set state validity checking for this space using Colision.h function isValidStatePoint
    ss.setStateValidityChecker([obstacles](const ob::State *state){return isValidStatePoint(state, obstacles);});
  
    // Set start and goal for simple setup
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    setStartAndGoal(start, goal, env);
    ss.setStartAndGoalStates(start, goal);
  
    // Output information
    ss.setup();
    ss.print();
  
    // Attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(ALLOWED_RUN_TIME);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
        ss.getSolutionPath().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

// Planning for Box Robot
void planBox(const std::vector<Rectangle> &obstacles, int env)
{
    // TODO: Use your implementation of RTP to plan for a rotating square robot.

    // SetUp Space: for box with x translation, y translation, and rotation -- SE2 Space
    auto space(std::make_shared<ob::SE2StateSpace>());
  
    // Set the bounds for the R^2 space
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-SPACE_DIM);
    bounds.setHigh(SPACE_DIM);
    space->setBounds(bounds);
  
    // Define a simple setup class
    og::SimpleSetup ss(space);

    // Set planner
    ss.setPlanner(std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation()));
    
    // Set state validity checking for this space using Colision.h function isValidStateSquare
    ss.setStateValidityChecker(std::bind(isValidStateSquare, std::placeholders::_1, SIDE_LEN, obstacles));
  
    // Set start and goal for simple setup
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    setStartAndGoal(start, goal, env);
    ss.setStartAndGoalStates(start, goal);
  
    // Output information
    ss.setup();
    ss.print();
  
    // Attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(ALLOWED_RUN_TIME);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
        ss.getSolutionPath().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

// Environment 1
void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    struct Rectangle upper = {-7, 0.75, 14, 9.25};
    struct Rectangle lower = {-7, -10, 14, 9.25};
    obstacles.push_back(upper);
    obstacles.push_back(lower);
}

// Environment 2
void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
    struct Rectangle upperObstacle = {-3,3, 4, 7};
    struct Rectangle lowerObstacle = {-3, -10, 4, 7};
    struct Rectangle endObstacle = {4, -7, 2, 14};

    obstacles.push_back(upperObstacle);
    obstacles.push_back(lowerObstacle);
    obstacles.push_back(endObstacle);
}

// Main Function
int main(int /* argc */, char ** /* argv */)
{
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;

        std::cin >> robot;
    } while (robot < 1 || robot > 2);

    do
    {
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) Hallway" << std::endl;
        std::cout << " (2) Walls" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    switch (choice)
    {
        case 1:
            makeEnvironment1(obstacles);
            break;
        case 2:
            makeEnvironment2(obstacles);
            break;
        default:
            std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            planPoint(obstacles, choice);
            break;
        case 2:
            planBox(obstacles, choice);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}
