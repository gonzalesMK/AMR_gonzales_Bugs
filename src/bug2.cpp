#include <obstacle_avoidance/bug2.h>
#include <obstacle_avoidance/math.h>

//Flag para o sensor frontal

namespace bug{
    /**
    *  Empty constructor
    */
    Bug2::Bug2(){
        // Reserve memory space for 3 sonars.
        for(int i=0; i<2; i++){
            sonarArray.push_back(0);    
        }
        
        
    }

    /**
    *  Empty destructor
    */
    Bug2::~Bug2(){;}
    
    /**
    * Go to Point: Computes desired twist that takes the robot towards the goal point. 
    */
    void Bug2::goToPoint(void){
          double yaw,delta,teta;
          geometry_msgs::Quaternion qt;  

          qt = this->odometry.pose.pose.orientation ;
          yaw = tf::getYaw(qt);
          double yi = this->odometry.pose.pose.position.y;
          double xi = this->odometry.pose.pose.position.x;
          double xf = this->goal.x;
          double yf = this->goal.y;
          teta = atan2((yf - yi) , ( xf - xi));
          delta = math::normalizeAngle((teta - yaw));

          if ( fabs(delta) > 0.2 ){
            this->twist.angular.z = delta * 3;
            this->twist.linear.x =(M_PI - fabs(delta))*1 ;
          } else {
            this->twist.angular.z = delta * 1 ;
            this->twist.linear.x = 1;
         }
    
    }
    
    
    /**
    * Wall Following function: Computes desired twist that allows robot circum-navagiating a wall/obstacle.
    */
    void Bug2::wallFollower(void){
        /** QUANDO APENAS O SENSOR DA FRENTE ACIONA, E O LATERAL AINDA NÃO FOI ACIONADO, ELE VIRA PARA A DIREITA E ANDA DEVAGAR
            QUANDO O SENSOR LATERAL ESQUERDO E FRONTAL ACIONAM JUNTOS, ELE VIRA PARA A DIRETA E ANDA DEVAGAR
            QUANDO O ESQUERDO ESTÁ PERTO E ACIONA SOZINHO, ELE VIRA PARA A DIREITA
            QUANDO O ESQUERDO ESTÁ LONGE E ACIONA SOZINHO, ELE VIRA PARA A ESQUERDA
            SENÃO, E SE O SENSOR DA FRENTE NÃO ESTIVER ACIONADO, VIRA PARA A ESQUERDA*/
     
        if(this->)
    /**
    * Bug Manager: Decides which sub-routine shall be called.
    */
    void Bug2::bugManager(void){
        // Static variables hold values
        static int state = 0; 
        static double shortestDistanceToGoal = 0;
        
        // Compute current distance in respect to the final goal 
        double distanceToGoal = math::distance2D(this->odometry.pose.pose.position, this->goal);
        // Compute current distance in respect to last hit point
        double distanceToH_in = math::distance2D(this->odometry.pose.pose.position, this->h_in);
        // Compute current distance in respect to last leaving point
        double distanceToH_out = math::distance2D(this->odometry.pose.pose.position, this->h_out);
        
        std::cout << "state: " << state << std::endl;
        std::cout << "Ponto Y: " << fabs(math::DeltaPointLine( this->odometry.pose.pose.position, this->goal, this->start)) << std::endl;

        
        switch (state){
            // State 0: Nothing to do.
            case 0: //Reseta a flag do sensor frontal
                    if (distanceToGoal > 0.1){ // Robot is far from the goal
                        // Change to "Go to point" state.
                        state = 1;
                    }
                    break;
                
            // State 1: Obstacle free, pursue the goal!
            case 1: // Move toward the goal.
                    this->goToPoint(); 
                    // Did the robot reach the goal?
                    if (distanceToGoal < 0.05){  
                        // Change to "resting" state.
                        this->twist.linear.x = 0;
                        this->twist.angular.z = 0;
                        state = 0;
                    } else
                    // Did the robot detected an obstacle in front of it? 
                    if (this->sonarArray[FRONT_SONAR] > 0 && this->sonarArray[FRONT_SONAR] < 0.8){
                        // Save hit IN point.
                        h_in = odometry.pose.pose.position;
                        // Change to "obstacle detected" state.
                        state = 2;
                    }
                    break;
            
            // State 2: The robot has just detected an obstacle.
            case 2: // Follow the wall.
                    this->wallFollower();
                    // Remain in this state until robot is far enough from hit point.
                    if (distanceToH_in  > 0.5){
                        // So far, this is shortest distance to goal.
                        shortestDistanceToGoal = distanceToGoal;
                        // Change "circum-navigate" state.
                        state = 3;
                    }
                    break;
            
            // State 3: The robot must circum-navigate the obstacle until Hm is found, or H_in
            case 3: // Follow the wall.
                    this->wallFollower();
                        //If returned to start, return failure
                        if(distanceToH_in < 0.2){
                            while(true);
                        }
                        //If found Hm, its continue to goToPoint
                        if (fabs(math::DeltaPointLine( this->odometry.pose.pose.position, this->goal, this->start))< 0.5)   
                            state = 0;                             
                    
                    break;
        }
        
    }
    
} // closing bug namespace







