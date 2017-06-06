#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <homework_5/MovementAction.h>

int main (int argc, char **argv) {
	ros::init(argc, argv, "movement_client");

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<homework_5::MovementAction> ac("movement", true);
	int dist, spd;

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	homework_5::MovementGoal goal;
	goal.desired_speed = 25.0;
	goal.distance = 1.0;
	ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	} else 
		ROS_INFO("Action did not finish before the time out.");

	//exit
	return 0;
}
