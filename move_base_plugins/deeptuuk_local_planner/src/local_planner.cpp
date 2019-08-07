#include <deeptuuk_local_planner/local_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Quaternion.h>

#include <math.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(deeptuuk::DeepTuukLocalPlanner, nav_core::BaseLocalPlanner)

namespace deeptuuk {
    void DeepTuukLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if(!initialized_){
            initialized_ = true;
            state_flag = 0;
        }
        else{
            ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
        }
    }

    bool DeepTuukLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        if(state_flag == 1){
            yaw_error_sum += yaw_error;
            cmd_vel.angular.z = 1.0f * yaw_error; //+ 0.01f * yaw_error_sum;
        }
        else if(state_flag == 2){
            cmd_vel.angular.z = 0;
        }
        return true;
    }

    bool DeepTuukLocalPlanner::isGoalReached()
    {
        if((abs(yaw_error) < 0.1f)&&(state_flag==1)){
            state_flag = 2;
            yaw_error_sum = 0;
            yaw_error = 0;
            return false;
        }
        return false;
    }

    bool DeepTuukLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        static double start_x=0;
        static double start_y=0;
        static double start_yaw=0;

        if(state_flag == 0){
            start_x = plan[0].pose.position.x;
            start_y = plan[0].pose.position.y;
            state_flag = 1;

            start_yaw = atan2( (plan[-1].pose.position.y - plan[0].pose.position.y), (plan[-1].pose.position.x - plan[0].pose.position.x) );
        }

        if(state_flag == 1){
            geometry_msgs::Quaternion msg;
            msg.x = plan[0].pose.orientation.x;
            msg.y = plan[0].pose.orientation.y;
            msg.z = plan[0].pose.orientation.z;
            msg.w = plan[0].pose.orientation.w;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(msg, quat);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  

            yaw_error = start_yaw - yaw;
            ROS_INFO("ttttttttt: %f, %f, %f",yaw_error,start_yaw,yaw);
        }
        return true;
    }

};