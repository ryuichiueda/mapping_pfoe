#include <ros/ros.h>
#include <tf/transform_listener.h>
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
int step = 0;

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

	ros::init(argc,argv,"navigator");
	ros::NodeHandle n;

	/*
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
	tf::TransformListener listener;

 	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	ROS_INFO("move_base action server OK");

	move_base_msgs::MoveBaseGoal goal;

	ros::Rate r(10);
	while(ros::ok()){
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time(0);
		listener.waitForTransform("odom", "map", goal.target_pose.header.stamp, ros::Duration(4.0));

		tf::StampedTransform transform;
		try{
			listener.lookupTransform("map", "odom", goal.target_pose.header.stamp, transform);
			cout << transform.getOrigin().x() << "\t" 
			     << transform.getOrigin().y() << endl;
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		double dx = trajectory[step].x - transform.getOrigin().x();
		double dy = trajectory[step].y - transform.getOrigin().y();

		cout << sqrt(dx*dx + dy*dy) << endl;
		if(sqrt(dx*dx + dy*dy) < 0.3 || ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			step = (step+1)%trajectory.size();

			goal.target_pose.pose.position.x = trajectory[step].x;
			goal.target_pose.pose.position.y = trajectory[step].y;
			goal.target_pose.pose.orientation.w = 0.1;
			ROS_INFO("Sending goal");
			for(int i=0;i<10;i++){
				ac.sendGoal(goal);
				sleep(0.1);
			}
		}else{
			ros::spinOnce();
			r.sleep();
		}
	
	
		ac.waitForResult();
	
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("FINISHED");
		else
			ROS_INFO("not reached");
	

	}
*/

	return 0;
}
