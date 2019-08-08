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
        private:
            bool initialized_= 0;
            int record_flag = 0;
            int state_flag = 0;
            int goal_reached_flag=0;

            double begin_x = 0;
            double begin_y = 0;
            double begin_yaw = 0;

            double end_x = 0;
            double end_y = 0;    
            double end_yaw = 0;        

            double exp_yaw = 0;

            double yaw_error = 0;
            double yaw_error_last = 0;

            double dis_error = 0;
            double dis_error_last = 0;

            double yaw_cur = 0;
    };
};

#endif