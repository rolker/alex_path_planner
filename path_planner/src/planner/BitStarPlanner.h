#ifndef SRC_BITSTARPLANNER_H
#define SRC_BITSTARPLANNER_H

#include "Planner.h"

// adapted in part from SamplingBasedPlanner.h
class BitStarPlanner : public Planner {
    public:
        BitStarPlanner();

        ~BitStarPlanner() override = default;

        /**
         * Plan using the provided planning problem and configuration. Guaranteed to return before timeRemaining has elapsed.
         * @param ribbonManager the ribbon manager
         * @param start the start state
         * @param config planner configuration
         * @param previousPlan previous plan to help seed search
         * @param timeRemaining computation time bound
         * @return
         */
        Stats plan(const RibbonManager&, const State& start, PlannerConfig config, const DubinsPlan& previousPlan, double timeRemaining) override;

        /**
         * Construct a single plan by tracing back from the given vertex to the root.
         * @param v
         * @param smoothing
         * @param obstacles
         * @return
         */
        DubinsPlan tracePlan(const std::shared_ptr<Vertex>& v, bool smoothing, const DynamicObstaclesManager& obstacles);

        /**
         * Manually set the planner config. Meant for testing.
         * @param config
         */
        void setConfig(PlannerConfig config);
    protected:
        void visualizePlan(const DubinsPlan& plan);
};


#endif // SRC_BITSTARPLANNER_H