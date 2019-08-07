#include <deeptuuk_global_planner/global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/Path.h>

#include <math.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(deeptuuk::DeepTuukGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace deeptuuk {

    void DeepTuukGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id)
    {
        if(!initialized_){
            ros::NodeHandle private_nh("~/" + name);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            frame_id_ = frame_id;
            initialized_ = true;
        }
        else{
            ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
        }
    }

    void DeepTuukGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
    }

    bool DeepTuukGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        plan.clear();

        double x1=start.pose.position.x;
        double y1=start.pose.position.y;

        double x2=goal.pose.position.x;
        double y2=goal.pose.position.y;

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = frame_id_;
        pose.pose.position.x = x1;
        pose.pose.position.y = y1;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = start.pose.orientation.x;
        pose.pose.orientation.y = start.pose.orientation.y;
        pose.pose.orientation.z = start.pose.orientation.z;
        pose.pose.orientation.w = start.pose.orientation.w;     
        
        plan.push_back(pose);

        double dis = hypot(x1-x2, y1-y2);
        double temp_sum = (dis/0.05);

        if( (x2 - x1 == 0) && (y2 - y1 == 0) ){
            return false;
        }
        else if( (x2 - x1 == 0) ){
            for(int i=1;i<temp_sum;i++){
                pose.pose.position.x = x1;
                pose.pose.position.y = y1 + (y2 -y1)/temp_sum*i; 
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                plan.push_back(pose);
            }
        }
        else if( (y2 - y1 == 0) ){
            for(int i=1;i<temp_sum;i++){
                pose.pose.position.x = x1 + (x2 -x1)/temp_sum*i;
                pose.pose.position.y = y1; 
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                plan.push_back(pose);
            }
        }
        else{
            double k = (y2 - y1)/(x2 - x1);
            double b = (y2 -k*x2);
            double temp = 0;

            for(int i=1;i<temp_sum;i++){
                temp = x1 + (x2 - x1)/temp_sum*i;
                //std::cout << std::setprecision(4) << temp << " " << std::setprecision(4) << k*temp+b <<" "<<0.0<< std::endl;
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = frame_id_;
                pose.pose.position.x = temp;
                pose.pose.position.y = k*temp+b;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                plan.push_back(pose);
            }
        }

        pose.pose.position.x = x2;
        pose.pose.position.y = y2;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = goal.pose.orientation.x;
        pose.pose.orientation.y = goal.pose.orientation.y;
        pose.pose.orientation.z = goal.pose.orientation.z;
        pose.pose.orientation.w = goal.pose.orientation.w;             

        plan.push_back(pose);

        nav_msgs::Path gui_path;
        gui_path.poses.resize(plan.size());

        gui_path.header.frame_id = frame_id_;
        gui_path.header.stamp = ros::Time::now();

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < plan.size(); i++) {
            gui_path.poses[i] = plan[i];
        }

        plan_pub_.publish(gui_path);
        return true;
    }



}

