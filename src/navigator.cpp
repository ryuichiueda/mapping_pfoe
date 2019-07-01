#include "ros/ros.h"
#include <ros/package.h> 
#include <iostream> 
#include <fstream> 
#include <iomanip> 
#include <vector> 
#include <string> 

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "visualization_msgs/MarkerArray.h"

using namespace std;

bool load_finished = false;
vector<geometry_msgs::Point> trajectory;

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

int main(int argc, char **argv)
{
	read_trajectory("/tmp/trajectory.tsv");
	return 0;

	ros::init(argc,argv,"navigator");
	ros::NodeHandle n;

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

//	Subscriber sub = n.subscribe("/trajectory_node_list", 1, cbTeaching);
	ros::spin();

	return 0;
}
