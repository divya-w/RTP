///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Austin Hushower and Divya Wagh
//////////////////////////////////////

#include "RTP.h"

// TODO: Implement RTP as described
#include <limits>
#include <stdlib.h>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

// RTP Constructor
ompl::geometric::RTP::RTP(const base::SpaceInformationPtr &si)
  :base::Planner(si, "RTP")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("goal_bias", this, &RTP::setGoalBias, &RTP::getGoalBias, "0.:.05:1.");
}

// RTP Deconstructor
ompl::geometric::RTP::~RTP()
{
    freeMemory();
}

// Clear Function
void ompl::geometric::RTP::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();

    if (motions.size() != 0)
    {
        motions.clear();
    }   

    lastGoalMotion_ = nullptr;
}

// Planner Setup Function
void ompl::geometric::RTP::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

// Free Memory Function
void ompl::geometric::RTP::freeMemory()
{
    if (nn_)
    {
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

// Solve Function
ompl::base::PlannerStatus ompl::geometric::RTP::solve(const base::PlannerTerminationCondition &ptc)
{
    // Check validity of planner
    checkValidity();

    // Get goal 
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    // Find starting states and add new motions to motions
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motions.push_back(motion);
    }

    if (motions.size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    // Sets pointers for sotution and approximate solution
    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    // Initalize state for random configuration space point (rstate)
    // Initialize state for later calculations (xstate)
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    while (!ptc)
    {
        // SELECT: select random configuration point qb from configuration space (set state to rstate)
        // Uses goal biasing
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        // SAMPLE: select random configuration point qa from existing tree (set motion to nmotion)
        // Generate random number between 0 and motions size
        // Select random element from motions vector
        int random = rand() % motions.size();
        Motion *nmotion = motions[random];

        // CHECK: whether the straight-line path between qa and qb in the C-space is valid and add to tree if so
        if (si_->checkMotion(nmotion->state, rstate))
        {
            // Builds tree
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, rstate);
            motion->parent = nmotion;
            motions.push_back(motion);
            nmotion = motion;

            double dist = 0.0;
            bool sat = goal->isSatisfied(nmotion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = nmotion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = nmotion;
            }
        }
    }

    // Approximate Solution
    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    // Solution
    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        // Construct the solution path
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // Set the solution path
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), motions.size());

    return {solved, approximate};
}

// Get Planner Data
void ompl::geometric::RTP::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}
