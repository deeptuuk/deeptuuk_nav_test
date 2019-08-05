#ifndef DEEPTUUK_GLOBAL_PLANNER_H_
#define DEEPTUUK_GLOBAL_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_core/base_global_planner.h>

namespace deeptuuk {
    class DeepTuukGlobalPlanner : public nav_core::BaseGlobalPlanner {
        public:
            DeepTuukGlobalPlanner(){}
            bool makePlan(const geometry_msgs::PoseStamped& start, 
                const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    };
};

#endif