#include <obstacle_avoidance/bug1.h>
#include <obstacle_avoidance/math.h>

//Flag para o sensor frontal
int flag = 0;

namespace bug{
    /**
    *  Empty constructor
    */
    Bug1::Bug1(){
        // Reserve memory space for 3 sonars.
        for(int i=0; i<2; i++){
            sonarArray.push_back(0);    
        }
        
        
    }

    /**
    *  Empty destructor
    */
    Bug1::~Bug1(){;}
    
    /**
    * Go to Point: Computes desired twist that takes the robot towards the goal point. 
    */
    void Bug1::goToPoint(void){
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
    void Bug1::wallFollower(void){
       double df,di; 
        /** QUANDO APENAS O SENSOR DA FRENTE ACIONA, E O LATERAL AINDA NÃO FOI ACIONADO, ELE VIRA PARA A DIREITA E ANDA DEVAGAR
            QUANDO O SENSOR LATERAL ESQUERDO E FRONTAL ACIONAM JUNTOS, ELE VIRA PARA A DIRETA E ANDA DEVAGAR
            QUANDO O ESQUERDO ESTÁ PERTO E ACIONA SOZINHO, ELE VIRA PARA A DIREITA
            QUANDO O ESQUERDO ESTÁ LONGE E ACIONA SOZINHO, ELE VIRA PARA A ESQUERDA
            SENÃO, E SE O SENSOR DA FRENTE NÃO ESTIVER ACIONADO, VIRA PARA A ESQUERDA*/
        df = this->sonarArray[LEFT_SONAR];
        if( this->sonarArray[FRONT_SONAR] < 0.5  && this->sonarArray[LEFT_SONAR]==0 && this->sonarArray[RIGHT_SONAR] == 0 && flag == 0 && this->sonarArray[FRONT_SONAR] != 0){
            this->twist.angular.z = -1;
            this->twist.linear.x =  0.5 ;
            std::cout << " Primeiro if" << std::endl;    
       }else if(this->sonarArray[FRONT_SONAR] != 0 && this->sonarArray[LEFT_SONAR]!=0){
            this->twist.angular.z = -1.5;
            this->twist.linear.x = 0.5 ; 
            std::cout << " Segundo if" << std::endl;
        }else if ( this->sonarArray[LEFT_SONAR]  < 0.4 && this->sonarArray[LEFT_SONAR]  !=0  ){
            // Se  Delta d for maior que zero , o carro acelerou para a direita, logo deve ser desacelarado para a esquerda
            if ( (df - di) > 0. )
                this->twist.angular.z = - (this->sonarArray[LEFT_SONAR] - 0.5 ) * 5 ;
            else {
                this->twist.angular.z = (this->sonarArray[LEFT_SONAR] - 0.5 ) * 5 ;
            }
            this->twist.linear.x =  2 ;
            flag = 1;
            std::cout << " Terceiro if" << std::endl;
        }else if( this->sonarArray[LEFT_SONAR]>0.45 ) {
            if ( (df - di) < 0 )
                this->twist.angular.z = - (this->sonarArray[LEFT_SONAR] - 0.4 ) * 5 ;
            else {
                this->twist.angular.z = (this->sonarArray[LEFT_SONAR] - 0.4 ) * 5 ;
            }
            this->twist.linear.x =  2 ;
            std::cout << " Quarto if" << std::endl;
        }else if (this->sonarArray[FRONT_SONAR] == 0 && this->sonarArray[LEFT_SONAR] == 0){
            std::cout << "Else" << std::endl;
            this->twist.angular.z = 3;
        }
        std::cout << "W: " << this->twist.angular.z << std::endl;
        std::cout << "ds: " << df -di << std::endl;
        std::cout << "Sonar: " << this->sonarArray[LEFT_SONAR] << std::endl;
        
        di = this->sonarArray[LEFT_SONAR];
    }
    
    /**
    * Bug Manager: Decides which sub-routine shall be called.
    */
    void Bug1::bugManager(void){
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
/*        std::cout << "distance to goal: " << distanceToGoal << std::endl;
        std::cout << "distance to hin: " << distanceToH_in << std::endl;
        std::cout << "distance to hout: " << distanceToH_out << std::endl;*/
        
        switch (state){
            // State 0: Nothing to do.
            case 0: //Reseta a flag do sensor frontal
                    flag=0;
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
            
            // State 3: The robot must circum-navigate the obstacle.
            case 3: // Follow the wall.
                    this->wallFollower();
                    // Is robot closer to the goal than ever before?
                    if(distanceToGoal < shortestDistanceToGoal){
                        // Yes! Then save current position.
                        this->h_out = this->odometry.pose.pose.position;
                        shortestDistanceToGoal = distanceToGoal;
                    }
                    // Remain in this state until robot is back to initial hit point (hit IN).
                    if (distanceToH_in < 0.4){
                        // Change "back to closest point" state.
                        state = 4;
                    }
                    break;
                
            // State 4: Take the robot back to the closest point in respect to final goal
            case 4: // Follow the wall.
                    this->wallFollower();
                    // Remain in this state until robot is back to leaving point (hit OUT).
                    if (distanceToH_out < 0.3){
                        // change state.
                        state = 0;
                    }
                    break;
                                        
        } 
        
    }
    
} // closing bug namespace







