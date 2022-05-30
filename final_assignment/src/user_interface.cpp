/**
* \file user_interface.cpp
* \brief User interface node for robot control
* \author Aurora Durante
* \version 1.0
* \date 04/02/2022
*
* \param [in] frame_id Define position of goal frame
* \param [in] w Define porientation of goal frame
*
* Publishes to: <BR>
*  °/move_base/goal
*  °/move_base/cancel
* Services: <BR>
*  °/service
*
* Description:
* This node allows communication between a user and the robot through a user interface.
* The user gives input to the robot that succesively executes related commands to move 
* in the environment pf the simulation.
**/

#include "ros/ros.h"
#include "final_srv/Callbacks_srv.h" // custom service '/service'
#include "move_base_msgs/MoveBaseActionGoal.h" // topic 'move_base/goal'
#include "actionlib_msgs/GoalID.h" // topic 'move_base/cancel'

ros::Publisher pub_goal; ///< Publisher to /move_base/goal topic
ros::Publisher pub_canc; ///< Publisher to /move_base/cancel topic
ros::ServiceClient client; ///< Client to /service custom service

final_srv::Callbacks_srv var; ///< Variable used to pass data between nodes with the custom service

void auto_move(){
/**
* \brief Robot moves autonomously
* \param target defines the goal position to reach once the user set its coordinates
* \param canc defines an empty object to publish to /move_base/goal/cancel topic if the goal is unreachable
* \param ui_canc defines the user intention wheter to cancel or not the current goal
*
* In this modality the robot moves autonomously in the environment to reach a goal passed as input by
* the user. The robot is also able to cancel the goal if it is unfeasible or if the user ask it to.
*/

	move_base_msgs::MoveBaseActionGoal target;
	actionlib_msgs::GoalID canc = {};
	char ui_canc;
	
	ROS_INFO("You chose modality 1! \n");
	
	client.call(var);
	
	// Ask the user a new target position
	ROS_INFO("Set x and y of new target position: ");
	std::cin >> target.goal.target_pose.pose.position.x >> target.goal.target_pose.pose.position.y;
	pub_goal.publish(target);
	
	// Drive the robot towards target
	while (var.response.s!=1){
		var.request.mod1 = 0;
		client.call(var);
	}
	
	/* Control if the target is unreachable and eventually cancel it, otherwise ask the user 
	   if he/she wants to cancel it*/ 
	if (var.response.re==1){ // target is unreachable
		ROS_INFO("Goal cannot be reached!");
		pub_canc.publish(canc);
	}
	else { // asking the user if target has to be cancelled
		ROS_INFO("Do you want to cancel this goal? (Y/N): ");
		std::cin >> ui_canc;
		if (ui_canc=='Y'){ // the user wants to cancel it
			pub_canc.publish(canc);
			ROS_INFO("Goal cancelled");
		}
		else
			ROS_INFO("Ok, going towards the goal");
	}
}

void drive_alone(){
/**
* \brief User controls the robot
* \param d defines the keyboard inputs sent by the user
* \param mod2 defines the control modality to which the robot has to switch
*
* In this modality the user is free to entirely control the robot by using the keyboard.
* He/she can decide to stop controlling the robot by entering 'q'.
*/
	
	ROS_INFO("You chose modality 2! \n");
	
	client.call(var);
	
	ROS_INFO("Use keyboard to move the robot:");
	while (var.response.d!='q'){
		var.request.mod2 = 1;
		client.call(var);
	}
}

void drive_assistance(){
/**
* \brief User assisted to control the robot 
* \param d defines the keyboard inputs sent by the user
* \param mod1, mod2 define the control modalities to which the robot has to switch
*
* In this modality the user is able to move the robot while is constantly controlled by
* robot itself that acquires information about the environment to avoid collisions. The
* user can decide to stop controlling the robot by entering 'q'.
*/
	
	ROS_INFO("You chose modality 3! \n");
	
	client.call(var);
	
	ROS_INFO("Use keyboard to move the robot:");
	while (var.response.d!='q'){
		var.request.mod1 = 0;
		var.request.mod2 = 1;
		client.call(var);
	}
}

void ui_decide(){
/**
* \brief Choose control modality
* \param mod defines which modality the user chooses
*
* In this function the user can decide which of the available modalities to use:
*  1) autonomously reach a (x,y) coordinate inserted by the user
*  2) let the user drive the robot with the keyboard
*  3) let the user drive the robot assisting them to avoid collisions
*/
	   
	int mod;
	
	ROS_INFO("Choose among these 3 modalities: \n 1) robot moves autonomously to goal. \n 2) drive the robot to goal. \n 3) drive the robot with assistance to goal.");
	
	std::cin >> mod;
	if (mod==1) // autonomously reach the target
		auto_move();
	else if (mod==2) // user drive the robot
		drive_alone();
	else if (mod==3) // user is assisted to avoid collisions
		drive_assistance();
	else 
		ROS_INFO("This modality doesn't exist.");
}

int main(int argc, char **argv){
/**
* \brief Main function of the node
* \param argc is input of the node
* \param argv is input of the node
*
* In this main function the node is initialized and setted to handle communication with ROS system.
* It also defines the publishers and clients needed for this node and it constantly ask the user inputs
* about what the robot should do.
*/
	
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "user_interface");
	ros::NodeHandle nh;

	// Define the publishers, subscribers and clients to needed topics
	pub_goal = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
	pub_canc = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
	client = nh.serviceClient<final_srv::Callbacks_srv>("/service");
	
	// Start the user interface
	while (ros::ok){
		ui_decide();
	}
	
	return 0;
}
