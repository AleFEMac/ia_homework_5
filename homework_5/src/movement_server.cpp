#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <homework_5/MovementAction.h>

class MovementAction {
	
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<homework_5::MovementAction> as_;
	std::string action_name_;

	// create messages that are used to published feedback/result
	homework_5::MovementFeedback feedback_;
	homework_5::MovementResult result_;

	//Variables
	ros::Subscriber sub;
	bool success;
	bool set;
	float x;
	float y;
	float speed;
	float distance;

public:

	MovementAction(std::string name) :
	as_(nh_, name, boost::bind(&MovementAction::executeCB, this, _1), false),
	action_name_(name) {
		success = false;
		set = false;
		distance = -1.0;
		sub = nh_.subscribe("odom", 10, &MovementAction::distanceCB, this);
		as_.start();
	}

	~MovementAction(void) {
	}
	
	void distanceCB(const nav_msgs::Odometry::ConstPtr& msg) {
		ROS_INFO("distanceCB\n");
		float cur_x, cur_y;
		cur_x = msg->pose.pose.position.x;
		cur_y = msg->pose.pose.position.y;
		if(!set) {
			x = cur_x;
			y = cur_y;
			set = true;
		}
		else if(distance >= 0) {
			if(sqrt(pow(cur_x - x,2) + pow(cur_y - y,2)) >= distance) {
				success = true;
				result_.odom_pose = *msg;
			}
		}
		
	}

	void executeCB(const homework_5::MovementGoalConstPtr &goal) {
		geometry_msgs::Twist cv_msg;
		
		speed = goal->desired_speed;
		distance = goal->distance;
		
		ros::Publisher pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

		while(!success) {
			cv_msg.linear.x = speed;
			pub.publish(cv_msg);

			if (as_.isPreemptRequested() || !ros::ok()) {
				ROS_INFO("%s: Preempted", action_name_.c_str());
				as_.setPreempted();
				success = false;
				break;
			}
		}
		
		if(success) {
		  ROS_INFO("%s: Succeeded", action_name_.c_str());
		  as_.setSucceeded(result_);
		}
		
		set = false;
		success = false;
	}

};


int main(int argc, char** argv) {
	ros::init(argc, argv, "movement_server");

	MovementAction movement("movement");
	ros::spin();

	return 0;
}
