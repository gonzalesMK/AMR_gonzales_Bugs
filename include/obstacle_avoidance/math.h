// ROS Libraries
#include <geometry_msgs/Point.h>


    namespace math{
    
    //! 3D distance between points p1 and p2
    double distance(geometry_msgs::Point &p1, geometry_msgs::Point &p2);

    //! 2D distance between points p1 and p2
    double distance2D(geometry_msgs::Point &p1, geometry_msgs::Point &p2);
    
    //! Normalize angle to -PI to +PI
    double normalizeAngle(double theta); 
        
    //! Return the Y - Ym coordinate of the Xactual in the line formed by points Actual and Goal
    double DeltaPointLine(geometry_msgs::Point &actual,geometry_msgs::Point &goal,geometry_msgs::Point &start);
        
    } // closing math namespace
