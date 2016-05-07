#include "ros/ros.h"
#include <obstacle_avoidance/bug2.h>

// ROS libraries
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


// Instantiate obstacle avoidance algorithm
bug::Bug2 mybug;

/**
* Front sonar  callback function. 
*/
void frontSonarCallback(const std_msgs::Float32ConstPtr& sonar){
    mybug.sonarArray[FRONT_SONAR] = sonar->data;
}

/**
* Right sonar callback function. 
*/
void rightSonarCallback(const std_msgs::Float32ConstPtr& sonar){
    mybug.sonarArray[RIGHT_SONAR] = sonar->data;
}

/**
* Left sonar callback function. 
*/
void leftSonarCallback (const std_msgs::Float32ConstPtr& sonar){
    mybug.sonarArray[LEFT_SONAR] = sonar->data;
}

/**
* Odometry callback function. 
*/
void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom){
    mybug.goal.x = 1.425;
    mybug.goal.y = 0.7;
    mybug.start.x = 1.425;
    mybug.start.y = -6.72;
    
    mybug.odometry = *odom;
    }
    

int main(int argc, char **argv){
    // Initialize ROS wtihin this node
    ros::init(argc,argv,"bug");
    
    // Declared node
    ros::NodeHandle nh;
    
    // Declare subscribers
    ros::Subscriber sub_front_sonar = nh.subscribe("/vrep/vehicle/frontSonar",1,frontSonarCallback);
    ros::Subscriber sub_left_sonar  = nh.subscribe("/vrep/vehicle/leftSonar" ,1,leftSonarCallback);
    ros::Subscriber sub_right_sonar = nh.subscribe("/vrep/vehicle/rightSonar",1,rightSonarCallback);
    ros::Subscriber sub_odometry    = nh.subscribe("/vrep/vehicle/odometry",1,odometryCallback);
    
    // Publish message
    ros::Publisher pub_twist = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    // Define ROS Loop rate
    ros::Rate loop_rate(100);
    
    // Define Goal

    // ROS Loop
    while(ros::ok()){
        // Publish here
        mybug.bugManager();
        pub_twist.publish(mybug.twist);
        // ROS routine
        ros::spinOnce();
        loop_rate.sleep();
    }
}
