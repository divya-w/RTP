///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Austin Hushower and Divya Wagh
//////////////////////////////////////

#include <iostream>

// Your random tree planner
#include "RTP.h"

// Other planners
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/rrt/RRT.h>

// Benchmarking and SE3 Environment
#include <ompl/tools/benchmark/Benchmark.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>

using namespace ompl;

// Benchmarking for Apartment
void benchmarkApartment(std::string &benchmark_name, app::SE3RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    benchmark_name = std::string("Apartment");

    // Load in robot and environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    // Set Start and Goal based on Apartment.cfg file
    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(241.81);
    start->setY(106.15);
    start->setZ(36.46);
    start->rotation().setAxisAngle(0., 0., -1., 3.12);

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(-31.19);
    goal->setY(-99.85);
    goal->setZ(36.46);
    goal->rotation().setAxisAngle(0., 0., -1., 3.12);

    // Set bounds based on Apartment.cfg file
    base::RealVectorBounds bounds(3);
    bounds.setHigh(0, 295.77);
    bounds.setHigh(1, 168.26);
    bounds.setHigh(2, 90.39);
    bounds.setLow(0, -73.76);
    bounds.setLow(1, -179.59);
    bounds.setLow(2, -0.03);
 
    // Setup Simple Setup
    setup.getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);
    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    // Runtime and memory settings
    runtime_limit = 50.0;   // Increased to 50 sec
    memory_limit = 1000.0;  // Set high because memory usage is not always estimated correctly
    run_count = 50;     // Set to 50 as instructed
}

// Benchmarking for Home
void benchmarkHome(std::string &benchmark_name, app::SE3RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    benchmark_name = std::string("Home");

    // Load in robot and environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_robot.dae";
    //std::string robot_fname = "/usr/local/share/ompl/resources/3D/Home_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_env.dae";
    //std::string env_fname = "/usr/local/share/ompl/resources/3D/Home_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    // Set Start and Goal based on Home.cfg file
    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(252.95);
    start->setY(-214.95);
    start->setZ(46.19);
    start->rotation().setAxisAngle(1., 0., 0., 0.);

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(262.95);
    goal->setY(75.05);
    goal->setZ(46.19);
    goal->rotation().setAxisAngle(1., 0., 0., 0.);

    // Set bounds based on Home.cfg file
    base::RealVectorBounds bounds(3);
    bounds.setHigh(0, 325.00);
    bounds.setHigh(1, 337.89);
    bounds.setHigh(2, 142.33);
    bounds.setLow(0, -383.80);
    bounds.setLow(1, -371.47);
    bounds.setLow(2, -0.20);

    // Setup Simple Setup
    setup.getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);
    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    // Runtime and memory settings from Home.cfg file
    runtime_limit = 50.0;   // Increased to 50 sec
    memory_limit = 10000.0;  // Set high because memory usage is not always estimated correctly
    run_count = 50;     // Set to 50 as instructed
    
}

// Overall benchmarking function
// Selects Apartment or Home based on users choice and runs benchmarking with RTP, RRT, PRM, and EST
int benchmarking(int benchmark_id)
{
    // Initialization of setup and benchmark parameters
    app::SE3RigidBodyPlanning setup;
    std::string benchmark_name;
    double runtime_limit, memory_limit;
    int run_count;

    // Which benchmark to run (Apartment or Home)
    if (benchmark_id == 1)
        benchmarkApartment(benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else if (benchmark_id == 2)
        benchmarkHome(benchmark_name, setup, runtime_limit, memory_limit, run_count);

    // Create the benchmark object and add all the planners we'd like to run
    tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    tools::Benchmark b(setup, benchmark_name);

    // Add our RTP Planner
    b.addPlanner(std::make_shared<geometric::RTP>(setup.getSpaceInformation()));   

    // Add RRT with range 25
    auto rrt1 = std::make_shared<geometric::RRT>(setup.getSpaceInformation());
    rrt1->setName("RRT Range 25.");
    rrt1->setRange(25.);
    b.addPlanner(rrt1);

    // Add PRM and EST
    b.addPlanner(std::make_shared<geometric::PRM>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::EST>(setup.getSpaceInformation()));

    // Benchmark
    b.benchmark(request);
    b.saveResultsToFile();

    return 0;
}

// Main
int main(int /* argc */, char ** /* argv */)
{
    int environment;

    do
    {
        std::cout << "Benchmark for: " << std::endl;
        std::cout << " (1) Apartment" << std::endl;
        std::cout << " (2) Home" << std::endl;

        std::cin >> environment;
    } while (environment < 1 || environment > 2);

    switch (environment)
    {
        case 1:
            benchmarking(1);
            break;
        case 2:
            benchmarking(2);
            break;
        default:
            std::cerr << "Invalid Environment!" << std::endl;
            break;
    }

    return 0;
}
