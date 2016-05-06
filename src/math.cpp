#include <obstacle_avoidance/math.h>

namespace math{
/**
* Distance: Computes the 3D distance betweem points p1 and p2.
*/
double distance(geometry_msgs::Point &p1, geometry_msgs::Point &p2){
    return sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2) + pow(p1.z-p2.z,2) );
}

/**
* Distance 2D: Computes the 2D distance betweem points p1 and p2.
*/
double distance2D(geometry_msgs::Point &p1, geometry_msgs::Point &p2){
    return sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2) );
}
    
/**
* Normalize angle: Return the normalized angle from -PI to PI
*/
    
double normalizeAngle(double theta){
    if (theta > M_PI){
        theta = theta - 2*M_PI;
    } else if (theta < -M_PI){
        theta = theta + 2*M_PI;
    }
    return theta;
}
/**
Return the value of the Y - Ym coordinate of the X actual point in the line formed by Points Start and Goal
*/
double DeltaPointLine (geometry_msgs::Point &actual,geometry_msgs::Point &goal,geometry_msgs::Point &start){
    double Y = (start.y - goal.y)/(start.x-goal.x) * (actual.x - goal.x);
   
    return (Y - goal.y) ;
}
    
}