#include <deeptuuk_global_planner/global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/Path.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(deeptuuk::DeepTuukGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace deeptuuk {

    void DeepTuukGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
    {
        if(!initialized_){
        //   costmap_ = costmap;
        //   global_frame_ = global_frame;
        //   planner_ = boost::shared_ptr<NavFn>(new NavFn(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()));

            ros::NodeHandle private_nh("~/" + name);

            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

            //private_nh.param("visualize_potential", visualize_potential_, false);

            //if we're going to visualize the potential array we need to advertise
        //   if(visualize_potential_)
        //     potarr_pub_.advertise(private_nh, "potential", 1);

        //   private_nh.param("allow_unknown", allow_unknown_, true);
        //   private_nh.param("planner_window_x", planner_window_x_, 0.0);
        //   private_nh.param("planner_window_y", planner_window_y_, 0.0);
        //   private_nh.param("default_tolerance", default_tolerance_, 0.0);

            //get the tf prefix
            //ros::NodeHandle prefix_nh;
            //tf_prefix_ = tf::getPrefixParam(prefix_nh);

            //make_plan_srv_ =  private_nh.advertiseService("make_plan", &NavfnROS::makePlanService, this);

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

    bool makePlan(const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {

        double x1=-18.21, y1=64.97, x2=-17.23, y2=61.45;
        double dis = hypot(x1-x2, y1-y2);
        double k = (y2 - y1)/(x2 - x1);
        double b = (y2 -k*x2);
        //std::cout << dis << std::endl;
        double temp_sum = (dis/0.1);
        double temp = 0;
        int i=0;
        for(i=0;i<=temp_sum;i++)
        {
            temp = x1 + (x2 - x1)/temp_sum*i;
            std::cout << std::setprecision(4) << temp << " " << std::setprecision(4) << k*temp+b <<" "<<0.0<< std::endl;
        }


        return true;
    }



}

