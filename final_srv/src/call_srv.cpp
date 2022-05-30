/**
* \file call_srv.cpp
* \brief Custom service to allow communications between nodes
* \author Aurora Durante
* \version 1.0
* \date 04/02/2022
*
* Services: <BR>
*  °/service
*
* Description:
* This custom service allows communications between nodes about different topics:
* - status of the goal
* - minimum distances from the nearest obstacle in three sectors
* - keyboard commmands sent in input by the user
* - controlling modality to swicth to
**/

#include "ros/ros.h"
#include "final_srv/Callbacks_srv.h"

bool callback_srv(final_srv::Callbacks_srv::Request &req, final_srv::Callbacks_srv::Response &res)
{
/**
* \brief Custom service for nodes communication
* \param req is the request fild of the custom service
* \param res is the response field of the custom service

*
* In this function the service handles communications between other nodes by passing values of variables
* or other information, such as which modality the user want to use
*/
   	
   	// Information about the status of the goal
	res.s = req.succ; 
	res.re = req.rej;
	
	// Information about the minimum distances on the right, in front and on the left
	res.ri = req.right;
	res.c = req.center;
	res.l = req.left;
	
	// Information about the keyboard command
	res.d = req.dir;
	
	// Information about which controlling modality to use
	res.m1 = req.mod1;
	res.m2 = req.mod2;
		
	return true;
}

int main(int argc, char **argv)
{
/**
* \brief Main function of the node
* \param argc is input of the node
* \param argv is input of the node
*
* In this main function the node is initialized and setted to handle communication with ROS system.
* It also defines the server needed for this node..
*/

	// Initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "call_srv");
	ros::NodeHandle ns;
	
	// Define the server for /service custom service
	ros::ServiceServer service = ns.advertiseService("/service", callback_srv);
	
	ros::spin();
	
	return 0;
}
