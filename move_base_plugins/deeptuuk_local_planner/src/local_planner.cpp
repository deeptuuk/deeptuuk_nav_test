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
            state_flag = 1;
        }
        else{
            ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
        }
    }

    bool DeepTuukLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        if(state_flag == 4){
            record_flag = 0;
            begin_x = 0;
            begin_y = 0;
            begin_yaw = 0;
            end_x = 0;
            end_y = 0;    
            end_yaw = 0;        
            exp_yaw = 0;
            yaw_error = 0;
            yaw_error_last = 0;
            dis_error = 0;
            dis_error_last = 0;
            yaw_cur = 0;    

            goal_reached_flag = 1;             
        }        

        if(state_flag == 3){
            yaw_error = end_yaw - yaw_cur;

            cmd_vel.angular.z = -(1.0f* yaw_error + 0.1f*(yaw_error - yaw_error_last)); //+ 0.01f * yaw_error_sum;
            if( (yaw_error < 0.06f) && (yaw_error > -0.06f) ){
                state_flag = 4;
                //ROS_INFO("1111111111");  
            }              
        }

        if(state_flag == 2){
            yaw_error = exp_yaw - yaw_cur;

            cmd_vel.angular.z = -(1.0f* yaw_error + 0.1f*(yaw_error - yaw_error_last));
            cmd_vel.linear.x = 0.4f*dis_error; //+ 0.01f*(dis_error - dis_error_last);

            cmd_vel.linear.x = ( (cmd_vel.linear.x > 0.5f) ? 0.5f:cmd_vel.linear.x );
            cmd_vel.linear.x = ( (cmd_vel.linear.x < -0.5f) ? -0.5f:cmd_vel.linear.x );

            if( (dis_error < 0.02f) && (dis_error > -0.02f) ){
                state_flag = 3;       
                //ROS_INFO("xxxxxxxx %f", yaw_error);           
            }
        }  

        if(state_flag == 1){
            yaw_error = exp_yaw - yaw_cur;

            cmd_vel.angular.z = -(1.0f* yaw_error + 0.1f*(yaw_error - yaw_error_last)); //+ 0.01f * yaw_error_sum;
            if( (yaw_error < 0.06f) && (yaw_error > -0.06f) ){
                state_flag = 2;
            }            
        }              

        yaw_error_last = yaw_error;
        dis_error_last = dis_error;        
        return true;
    }

    bool DeepTuukLocalPlanner::isGoalReached()
    {
        if(goal_reached_flag == 1){
            goal_reached_flag = 0;
            state_flag = 1;
            return true;
        }
        return false;
    }

    bool DeepTuukLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        if(record_flag == 0){
            begin_x = plan[0].pose.position.x;
            begin_y = plan[0].pose.position.y;

            geometry_msgs::Quaternion msg;
            msg.x = plan[0].pose.orientation.x;
            msg.y = plan[0].pose.orientation.y;
            msg.z = plan[0].pose.orientation.z;
            msg.w = plan[0].pose.orientation.w;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(msg, quat);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); 

            begin_yaw = yaw;             

            end_x = plan[plan.size()-1].pose.position.x;
            end_y = plan[plan.size()-1].pose.position.y;

            msg.x = plan[plan.size()-1].pose.orientation.x;
            msg.y = plan[plan.size()-1].pose.orientation.y;
            msg.z = plan[plan.size()-1].pose.orientation.z;
            msg.w = plan[plan.size()-1].pose.orientation.w;            
            tf::quaternionMsgToTF(msg, quat);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); 

            end_yaw = yaw;

            //ROS_INFO(" end_yaw: %f",end_yaw);

            exp_yaw = atan2(end_y - begin_y, end_x - begin_x);

            record_flag = 1;
        }

        if(record_flag == 1){
            geometry_msgs::Quaternion msg_tmp;
            msg_tmp.x = plan[0].pose.orientation.x;
            msg_tmp.y = plan[0].pose.orientation.y;
            msg_tmp.z = plan[0].pose.orientation.z;
            msg_tmp.w = plan[0].pose.orientation.w;
            tf::Quaternion quat_tmp;
            tf::quaternionMsgToTF(msg_tmp, quat_tmp);
            double roll_tmp, pitch_tmp, yaw_tmp;
            tf::Matrix3x3(quat_tmp).getRPY(roll_tmp, pitch_tmp, yaw_tmp);    

            dis_error = hypot(plan[0].pose.position.x - end_x, plan[0].pose.position.y - end_y);      

            yaw_cur = yaw_tmp;
        }
        return true;
    }
};
