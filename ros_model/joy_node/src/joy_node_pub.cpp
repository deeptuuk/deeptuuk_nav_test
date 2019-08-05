#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist temp;
geometry_msgs::Twist temp_last;

void write_callback(const sensor_msgs::Joy::ConstPtr &joy_cmd)
{
	temp.linear.x = joy_cmd->axes[4];
	temp.angular.z = -joy_cmd->axes[3];
}

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "joy_node");
	ros::NodeHandle n;
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 0);
	ros::Subscriber joy_sub = n.subscribe("/joy", 0, write_callback);

	ros::Rate loop_rate(100);

	while (ros::ok())
	{
		ros::spinOnce();
		cmd_pub.publish(temp);

		loop_rate.sleep();

	}

	return 0;
}