// ROS Libraries
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
// C++ Libraries
#include <vector>

#define FRONT_SONAR 0
#define RIGHT_SONAR 1
#define LEFT_SONAR  2
#define SONAR_MAX_VAL 0.65
namespace bug{

class Bug2{
    public:
        // To-be received messages
        std::vector<double> sonarArray;     //!< Array for sonar data.
        nav_msgs::Odometry odometry;        //!< Odometry data.
        // To-be published messages
        geometry_msgs::Twist twist;
        // Bug variables
        geometry_msgs::Point goal;          //!< Goal point.
        geometry_msgs::Point h_in;          //!< Obstacle hit point.
        geometry_msgs::Point h_out;         //!< Obstacle leave point.
        geometry_msgs::Point start;  
    public:
        //! Empty constructor
        Bug2();
        //! Empty desructor
        ~Bug2();

        //! Go to point
        void goToPoint(void);

        //! WAll follower controller
        void wallFollower(void);
    
        //! Bug algorithm manager
        void bugManager(void);
    
}; // closing Bug1 class

}  // closing bug namespace
