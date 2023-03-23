//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"

// Include here the ".h" files corresponding to the topic types you use.
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

// You may have a number of globals here.

// Our marker message has many fields that remain constant and
// are initialized only once by the following function.

ros::Publisher pubMarker ;
visualization_msgs::Marker marker;

void initializeMarker(){
    marker.header.frame_id = "map";
    marker.ns = ros::this_node::getName();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5 ;
    marker.scale.y = 0.5 ;
    marker.scale.z = 0.02;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
}


// Function to publish a marke at a given (x,y) position.

void publishMarkerAt( geometry_msgs::PointStamped markerPos) {    
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = markerPos.point.x ;
    marker.pose.position.y = markerPos.point.y ;
    marker.pose.position.z = markerPos.point.z ;
    marker.lifetime = ros::Duration();
    pubMarker.publish(marker);
}


// Velocity callback function
geometry_msgs::Vector3Stamped lastVelocity ;
void velocityCallback(geometry_msgs::Vector3Stamped desiredVelocity){
    lastVelocity = desiredVelocity ;
}

int main (int argc, char** argv)
{

    //ROS Initialization
    ros::init(argc, argv, "drones_node");

    // Define your node handles
    ros::NodeHandle nh_loc("~") ;
    ros::NodeHandle nh_glob ;

    // Declare your node's subscriptions and service clients
    ros::Subscriber subVel = nh_glob.subscribe<geometry_msgs::Vector3Stamped>("vel_cmd",1,velocityCallback);

    // Declare you publishers and service servers
                   pubMarker = nh_glob.advertise<visualization_msgs::Marker>("/visualization_marker",1) ;
    ros::Publisher pubPos    = nh_glob.advertise<geometry_msgs::PointStamped>("drone_pos",1);

    // Drone position. Initially at origin.
    geometry_msgs::PointStamped dronePos ;
    dronePos.header.stamp = ros::Time::now() ;
    dronePos.header.frame_id = "map" ;
    dronePos.point.x = dronePos.point.y = dronePos.point.z = 0.0 ;
    
    // Drone velocity, initially zero. Will be updated by callback function.
    lastVelocity.header.stamp = ros::Time::now() ;
    lastVelocity.header.frame_id = "map" ;
    lastVelocity.vector.x = lastVelocity.vector.y = lastVelocity.vector.z = 0.0 ;

    initializeMarker() ;
    publishMarkerAt( dronePos ) ;

    ros::Rate rate(10);
    ros::Time currentTime, prevTime = ros::Time::now() ; 
    while (ros::ok()){

	    ros::spinOnce();

        currentTime = ros::Time::now() ;
        ros::Duration timeElapsed = currentTime - prevTime ;
        prevTime = currentTime ;
        double deltaT = timeElapsed.toSec() ;

        // Integrate speed. Velocity vector is updated by callback function.

        dronePos.header.stamp = currentTime ;
        dronePos.point.x += deltaT*lastVelocity.vector.x ;
        dronePos.point.y += deltaT*lastVelocity.vector.y ;
        dronePos.point.z += deltaT*lastVelocity.vector.z ;

        // Publish a marke at the position of the master.
        publishMarkerAt( dronePos ) ;
        pubPos.publish(dronePos) ;

	    rate.sleep();

    }
}

