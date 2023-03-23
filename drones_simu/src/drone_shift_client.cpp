/**
 * \file
 * \brief 
 * \author 
 * \version 0.1
 * \date 
 * 
 * \param[in] 
 * 
 * Subscribes to: <BR>
 *    ° 
 * 
 * Publishes to: <BR>
 *    ° 
 *
 * Description
 *
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"

// Include here the ".h" files corresponding to the topic type you use.
// ...

// You may have a number of globals here.
//...

// Callback functions...

void myFirstCallback(/* Define here the variable which hold the message */){
    // ... Callback function code
}


int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "drone_shit_client");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh_glob, nh_loc("~") ;

    // Read the node parameters if any
    // ...

    // Declare your node's subscriptions and service clients
    // Something like this:
    // ros::Subscriber subName = node_handle.subscribe<package_name::TypeName>("topic_name",buffer size often 1,callback_function) ;

    // Declare you publishers and service servers
    // Something like this:
    // ros::Publisher pubName = node_handle.advertise<package_name::Typename>("topic_name",buffer size often 1) ;

    // This node does not need to be executed at any specific frequency: no rate object.
    while (ros::ok()){
        ros::spinOnce();

        // Suggestion: read the data to create the message (id, rho, polar angle) from keyboard,
        // create the message and call the service.

    }
}

