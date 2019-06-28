#include "ros/ros.h"
#include <ros/package.h> 
#include <iostream> 
#include <fstream> 
#include <iomanip> 

#include "visualization_msgs/MarkerArray.h"

using namespace ros;

bool finished = false;

void cb(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	if(finished)
		return; 

	std::ofstream f("/tmp/trajectory.tsv"); 
	for(auto p : msg->markers[0].points ){ 
		f << std::fixed;
		f << std::setprecision(3) << p.x << "\t" << p.y << std::endl;
	}
	finished = true;
	ROS_INFO("trajectory extracted");
}

int main(int argc, char **argv)
{
	init(argc,argv,"trajectory_extractor");
	NodeHandle n;

	Subscriber sub = n.subscribe("/trajectory_node_list", 1, cb);
	spin();

	return 0;
}
