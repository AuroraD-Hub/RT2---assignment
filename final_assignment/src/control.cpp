/**
* \file control.cpp
* \brief Node for robot control
* \author Aurora Durante
* \version 1.0
* \date 04/02/2022
*
* \param [in] frame_id Define position of goal frame
* \param [in] w Define porientation of goal frame
*
* Publishes to: <BR>
*  °/cmd_vel
* Services: <BR>
*  °/service
*
* Description:
* This node lets the robot move in the environment with respect to which modality the user
* chooses. In any case velocity inputs are published in the topic so that the robot can move.
**/

#include "ros/ros.h"
#include "final_srv/Callbacks_srv.h"
#include "geometry_msgs/Twist.h" // topic 'cmd_vel'

ros::Publisher pub_vel; ///< Publisher to /cmd_vel topic
ros::ServiceClient client; ///< Client to /service custom service

final_srv::Callbacks_srv var; ///< Variable used to pass data between nodes with the custom service

void drive(){
/**
* \brief Autonomously moving in the environment
* \param min_th defines the threshold distance before that the robot collides with some obstacle
* \param vel defines the linear or angular velocity to publish with respct to where the obstacles are

* In this function the robot moves in the environment while avoiding obstacles: if there is an obstacle
* in front of, on the right or on the left, the robot turns in such a way to avoid it.
*/
	
	double min_th=0.5; // minimum distance threshold from the obstacles	
	geometry_msgs::Twist vel;
	
	client.call(var);
	
	if (var.response.c<min_th){ // obstacle in front of the robot
		ROS_INFO("Cannot go further this way!");
		vel.linear.x = 0;
		vel.angular.z = -1;
	}
	else{
		if (var.response.ri<min_th){ // obstacle to the right of the robot
			ROS_INFO("Cannot go this way! Turning left...");
			vel.linear.x = 0;
			vel.angular.z = 2;
		}
		else if (var.response.l<min_th){ // obstacle to the left of the robot
			ROS_INFO("Cannot go this way! Turning right...");
			vel.linear.x = 0;
			vel.angular.z = -2;
		}
	}
	pub_vel.publish(vel);
}

void keyboard_drive(){
/**
* \brief Moving with user inputs
* \param vel defines the linear or angular velocity to publish with respct to user inputs
* \param dir defines the input command of the user
*
* In this function user drives the robot in the environment by keyboard command: by pressing 
* 'u' it turns left, by pressing 'i' it gos foward, by pressing 'o' it turns right and by 
* pressing 'k' it stops. A linear/angular velocity is published accordingl to input command.
*/
	
	geometry_msgs::Twist vel;
	
	ROS_INFO("Press 'u' to turn left. \n Press 'i' to go foward \n Press 'o' to turn right \n Press 'k' to stop \n Press 'q' to quit this modality");
	std::cin >> var.request.dir;
	
	client.call(var);
	
	if (var.response.d=='u'){ // turn left
		vel.linear.x = 0;
		vel.angular.z = 1;
	}
	else if (var.response.d=='i'){ // go foward
		vel.linear.x = 1;
		vel.angular.z = 0;
	}
	else if (var.response.d=='o'){ // turn right
		vel.linear.x = 0;
		vel.angular.z = -1;
	}
	else if (var.response.d=='k'){ // stop
		vel.linear.x = 0;
		vel.angular.z = 0;
	}
	
	pub_vel.publish(vel);
}

int main(int argc, char **argv){
/**
* \brief Main function of the node
* \param argc is input of the node
* \param argv is input of the node
*
* In this main function the node is initialized and setted to handle communication with ROS system.
* It also defines the publishers and clients needed for this node and it chooses the control modality
* to use accordingly to m1 and m2 values.
*/
	
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "driver");
	ros::NodeHandle nh;

	// Define the publishers, subscribers and clients to needed topics
	pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	client = nh.serviceClient<final_srv::Callbacks_srv>("/service");
	
	// Start the user interface
	while(ros::ok()){
		client.call(var);
		if (var.response.m1==0)
			drive();
		else if (var.response.m2==1)
			keyboard_drive();
		ros::spinOnce();
	}
	
	return 0;
}
