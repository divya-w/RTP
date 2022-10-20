///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Austin Hushower and Divya Wagh
//////////////////////////////////////

#ifndef RANDOM_TREE_H
#define RANDOM_TREE_H

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>

namespace ompl
{
    namespace geometric
    {
        // TODO: Implement RTP as described

        class RTP : public base::Planner
        {
        public:
            // Constructor
            RTP(const base::SpaceInformationPtr &si);

            // Deconstructor
            ~RTP() override;

            // Get Planner Data function
            void getPlannerData(base::PlannerData &data) const override;

            // Solve function
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            // Clear function
            void clear() override;

            // Set Goal Bias
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            // Get Goal Bias
            double getGoalBias() const
            {
                return goalBias_;
            }

            // Set new nearest neighbors data strcuture (Delete? Will use NN as graph to keep track but no distance function)
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

        protected:
            // Representation of a motion (Contains pointers to parent motions as we only need to go backwards in the tree)
            class Motion
            {
            public:
                Motion() = default;
                // Constructor that allocates memory for the state
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }
                // Deconstructor
                ~Motion() = default;
                // The state contained by the motion
                base::State *state{nullptr};
                // The parent motion in the exploration tree
                Motion *parent{nullptr};
            };

            // Free the memory allocated by this planner
            void freeMemory();

            // nn_ Stuff for free memory function (ignore)
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;
            double maxDistance_{0.};
            
            // State sampler
            base::StateSamplerPtr sampler_;

            // Array datastructure to build and store Tree
            std::vector<Motion *> motions;

            // The fraction of time the goal is picked as the state to expand towards (if such a state is available)
            double goalBias_{.05};

            // The random number generator
            RNG rng_;

            // The most recent goal motion (used for PlannerData computation)
            Motion *lastGoalMotion_{nullptr};
        };

    }  // namespace geometric
}  // namespace ompl

#endif
