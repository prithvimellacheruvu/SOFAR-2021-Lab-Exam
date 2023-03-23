// Display a "drone" in an helicoidal motion and publish its position.

//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <vector>

#include "ros/ros.h"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <drones_msgs/RelativePos.h>
#include <drones_msgs/DroneGoals.h>
#include <drones_serv/DesiredRelativePos.h>

ros::Publisher pubMarker ;
visualization_msgs::Marker marker;

void initializeMarker(){
    // Create a marker for this node. Only timestamp and position will be later updated.
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
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
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

// Characteristics of leader trajectory.
double horizSpeed = 1.0, radius = 3.0 ;

geometry_msgs::PointStamped updateLeaderPosition()
{
           ros::Time currentTime = ros::Time::now() ;
    static ros::Time prevTime ;
    static geometry_msgs::PointStamped leaderPosition ;
    static double polarAngle = 0.0 ;

    ros::Duration timeElapsed = currentTime - prevTime ;
    prevTime = currentTime ;
    polarAngle += (horizSpeed/radius)*timeElapsed.toSec() ;
    leaderPosition.header.stamp = currentTime ;
    leaderPosition.point.x = radius*cos(polarAngle) ;
    leaderPosition.point.y = radius*sin(polarAngle) ;
    leaderPosition.point.z = 2 + 2*sin(2*M_PI/20*currentTime.toSec()) ;
    return leaderPosition ;
}

int nbDrones ; // Is set by reading a node parameter in the init.
std::vector<drones_msgs::RelativePos> relPositions ;

#define MAX_RHO 3.0
bool droneShiftCallback(drones_serv::DesiredRelativePos::Request  &req, drones_serv::DesiredRelativePos::Response &res )
{
    // Return error on incorrect drone id
    if( req.relPos.id > nbDrones-1 || req.relPos.id < 0 ){
        ROS_ERROR_STREAM("Drone shift service - Incorrect drone ID: " << req.relPos.id) ;
        return false ;
    }
    if( req.relPos.rho < 0.0 ) {
        ROS_ERROR_STREAM("Drone shift service - Incorrect distance: " << req.relPos.rho) ;
        return false ;
    }
    res.appliedRelPos.id = req.relPos.id ;
    res.appliedRelPos.rho = req.relPos.rho ;
    if( req.relPos.rho > MAX_RHO ){
        ROS_INFO_STREAM("Drone shift service - Distance too large: " << req.relPos.rho << " set to: " << MAX_RHO) ;
        res.appliedRelPos.rho = MAX_RHO ;
    }
    res.appliedRelPos.polarAngle = req.relPos.polarAngle ;
    relPositions[req.relPos.id] = res.appliedRelPos ;
    relPositions[req.relPos.id].polarAngle *= (M_PI/180.0) ;
    return true ;
}

int main (int argc, char** argv)
{

    //ROS Initialization
    ros::init(argc, argv, "leader");

    // Define your node handles
    ros::NodeHandle nh_loc("~") ;
    ros::NodeHandle nh_glob ;

    // Read the number of drones, a global parameter.
    std::string nodeName = ros::this_node::getName() ;
    nh_loc.param("nbDrones",nbDrones,1) ;
    ROS_INFO_STREAM(nodeName << ": nbDrones : " << nbDrones) ;

    // Declare you publishers and service servers
                       pubMarker  = nh_glob.advertise<visualization_msgs::Marker>("/visualization_marker",1) ;
    ros::Publisher     pubGoals   = nh_glob.advertise<drones_msgs::DroneGoals>("/goals",1);
    ros::ServiceServer droneShift = nh_glob.advertiseService("/drone_shift", droneShiftCallback) ;

    // Randomly initialize the positions of the followers relative to leader (polar coordinates).
    srand(time(NULL)) ;
    drones_msgs::RelativePos relPos ;
    std::vector<bool> inUse ;
    for(int i = 0 ; i < nbDrones ; i++){
        relPos.id = i ;
        relPos.rho = 0.5+1.0*rand()/RAND_MAX ; // distance 0.5 to 1.5
        relPos.polarAngle = -M_PI + 2*M_PI*(1.0*rand())/RAND_MAX ; // polar angle -pi to pi
        relPositions.push_back(relPos);
        inUse.push_back(true) ;   // All drones initially used.
    }

    // Initialize leader position
    geometry_msgs::PointStamped leaderPosition ;
    leaderPosition.header.frame_id = "map" ;
    leaderPosition.header.stamp = ros::Time::now() ;
    leaderPosition.point.x = radius ;  // global variable.
    leaderPosition.point.y = 0.0    ;
    leaderPosition.point.z = 0.0    ;

    initializeMarker() ;
    publishMarkerAt( leaderPosition ) ;

    ros::Rate rate(50);  

    while (ros::ok()){

        ros::spinOnce();

        leaderPosition = updateLeaderPosition() ;

        // Calculate and publish the goal points of the followers
        drones_msgs::DroneGoals followers ;
        for( int i = 0 ; i < nbDrones ; i++ ){
            followers.ids.push_back(i) ;
            geometry_msgs::PointStamped p ;
            p.header.stamp = ros::Time::now() ;
            p.header.frame_id = "map" ;
            p.point.x = leaderPosition.point.x + relPositions[i].rho*cos(relPositions[i].polarAngle) ;
            p.point.y = leaderPosition.point.y + relPositions[i].rho*sin(relPositions[i].polarAngle) ;
            p.point.z = leaderPosition.point.z ;
            followers.goals.push_back(p);
        }
        pubGoals.publish(followers) ;
        
        // Publish an  marker at the position of the master.
        publishMarkerAt( leaderPosition ) ;
        
	    rate.sleep();
    }
}

