#ifndef DEEPTUUK_LOCAL_PLANNER_H_
#define DEEPTUUK_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_core/base_local_planner.h>
#include <tf/transform_listener.h>

namespace deeptuuk {
    class DeepTuukLocalPlanner : public nav_core::BaseLocalPlanner {
        public:
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
            bool isGoalReached();
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
            void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
        protected:
            bool initialized_;
            int state_flag;
            double yaw_error;
            double yaw_error_last;
            double yaw_error_sum=0;
    };
};

#endif