#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ros/package.h> 
#include <iostream> 
#include <fstream> 
#include <iomanip> 
#include <vector> 
#include <string> 

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

#include "visualization_msgs/MarkerArray.h"

using namespace std;

bool load_finished = false;
vector<geometry_msgs::Point> trajectory;
int step = 0;

double x = 0.0;
double y = 0.0;
double yaw = 0.0;

bool read_trajectory(string filename)
{
	ifstream f(filename);
	if(!f)
		return false;

	double x,y;
	while(f >> x >> y){
		geometry_msgs::Point p;
		p.x = x;
		p.y = y;
		p.z = 0.0;
		trajectory.push_back(p);
	}

	if(trajectory.size() == 0)
		return false;

	int n = 0;
	for(auto p : trajectory){
		ROS_INFO("point %d\t%f\t%f", n++, p.x, p.y);
	}

	return true;
}

void cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	x = msg->pose.pose.position.x; 
	y = msg->pose.pose.position.y; 
	yaw = tf::getYaw(msg->pose.pose.orientation)/3.141592*180;
}


int main(int argc, char **argv)
{
	read_trajectory("/tmp/trajectory.tsv");

	ros::init(argc,argv,"navigator");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/amcl_pose", 1, cb);
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	geometry_msgs::Twist tw;
	tw.linear.x = 0.0;
	tw.angular.z = 0.0;

	int step = 0;
	int prev_step = trajectory.size() - 1;
	ros::Rate r(10);
	while(ros::ok()){
		double prev_x = trajectory[prev_step].x;
		double prev_y = trajectory[prev_step].y;
		double target_x = trajectory[step].x;
		double target_y = trajectory[step].y;

		double target_r = sqrt((target_x-x)*(target_x-x) + (target_y-y)*(target_y-y));

		if(target_r < 0.1){
			prev_step = step;
			step = (step+1)%trajectory.size();
			continue;
		}

		double target_direction = atan2(target_y-y, target_x-x)/3.141592*180 - yaw;

		if(target_direction > 180.0) target_direction -= 360.0;
		if(target_direction < -180.0) target_direction += 360.0;

		if( (fabs(target_direction) > 5.0 and target_r < 0.2) or fabs(target_direction) > 30.0 ){
			tw.linear.x /= 1.2;
		}else{
			tw.linear.x = 0.2;
		}

		tw.angular.z = 2*target_direction*3.141592/180;
		if(tw.angular.z > 2.0)       tw.angular.z = 2.0;
		else if(tw.angular.z < -2.0) tw.angular.z = -2.0;

		pub.publish(tw);

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
