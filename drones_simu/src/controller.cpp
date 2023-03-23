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

#define KP 10.0

int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "controller");

    // Node handles
    ros::NodeHandle nh_glob, nh_loc("~") ;

    // Read the node parameters if any
    // ...

    // Declare your subscribers and service clients
    // ros::Subscriber subName = node_handle.subscribe<package_name::TypeName>("topic_name",buffer size often 1,callback_function) ;

    // Declare your publishers, service servers, tf broadcasters...
    // ros::Publisher pubName = node_handle.advertise<package_name::Typename>("topic_name",buffer size often 1) ;
    
    ros::Rate rate(10);  // Or other frequency.

    while (ros::ok()){
        ros::spinOnce();

        // Code of your node goes here...
        // ...
        
        rate.sleep();
    }
}

