//author  Renato Sousa, 2018
//#include <QtNetwork>
#include <stdio.h>
#include <iostream>
#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"
#include "util/util.h"
#include <math.h>

class Objective{
    double m_x;
    double m_y;
    double m_angle;

public:
    Objective(double t_x, double t_y, double t_angle): m_x(t_x),m_y(t_y),m_angle(t_angle){};


    void setY(double value){
        m_y = value;
    }

    void setAngle(double value){
        m_angle = value;
    }

    void setX(double value){
        m_x = value;
    }
    double x(){
        return m_x;
    }
    double y(){
        return m_y;
    }
    double angle(){
        return m_angle;
    }
};


void printRobotInfo(const fira_message::sim_to_ref::Robot & robot) {

    printf("ID=%3d \n",robot.robot_id());

    printf(" POS=<%9.2f,%9.2f> \n",robot.x(),robot.y());
    printf(" VEL=<%9.2f,%9.2f> \n",robot.vx(),robot.vy());

    printf("ANGLE=%6.3f \n",robot.orientation());
    printf("ANGLE VEL=%6.3f \n",robot.vorientation());
}

double to180range(double angle) {
  angle = fmod(angle, 2 * M_PI);
  if (angle < -M_PI) {
    angle = angle + 2 * M_PI;
  } else if (angle > M_PI) {
    angle = angle - 2 * M_PI;
  }
  return angle;
}

double smallestAngleDiff(double target, double source) {
  double a;
  a = fmod(target + 2*M_PI, 2 * M_PI) - fmod(source + 2*M_PI, 2 * M_PI);

  if (a > M_PI) {
    a = a - 2 * M_PI;
  } else if (a < -M_PI) {
    a = a + 2 * M_PI;
  }
  return a;
}


void PID(fira_message::sim_to_ref::Robot robot, Objective objective, int index,GrSim_Client* grSim_client)
{
    double Kp = 20;
    double Kd = 2.5;
    static double lastError = 0;

    double rightMotorSpeed;
    double leftMotorSpeed;

    bool reversed = false;

    double angle_rob = robot.orientation();


    double angle_obj = atan2( objective.y() - robot.y(),  objective.x() - robot.x()) ;


    double error = smallestAngleDiff(angle_rob, angle_obj);

    if(fabs(error) > M_PI/2.0 + M_PI/20.0) {
          reversed = true;
          angle_rob = to180range(angle_rob+M_PI);
          // Calculates the error and reverses the front of the robot
          error = smallestAngleDiff(angle_rob,angle_obj);
    }

    double motorSpeed = (Kp*error) + (Kd * (error - lastError));// + 0.2 * sumErr;
    lastError = error;



    double baseSpeed = 30;

    // Normalize
    motorSpeed = motorSpeed > 30 ? 30 : motorSpeed;
    motorSpeed = motorSpeed < -30 ? -30 : motorSpeed;

    if (motorSpeed > 0) {
      leftMotorSpeed = baseSpeed ;
      rightMotorSpeed = baseSpeed - motorSpeed;
    } else {
      leftMotorSpeed = baseSpeed + motorSpeed;
      rightMotorSpeed = baseSpeed;
    }


    if (reversed) {
      if (motorSpeed > 0) {
        leftMotorSpeed = -baseSpeed + motorSpeed;
        rightMotorSpeed = -baseSpeed ;
      } else {
        leftMotorSpeed = -baseSpeed ;
        rightMotorSpeed = -baseSpeed - motorSpeed;
      }
    }
    grSim_client->sendCommand(leftMotorSpeed,rightMotorSpeed, false, index);
}

//First we define some functions that will be useful for us


double getDistance(double x1, double y1, double x2, double y2){
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}
double getAngle(double x1 , double y1,double x2, double y2){
  double Dx = x1 - x2;
  double Dy = y1 - y2;
  Dx =  abs(Dx);
  Dy =  abs(Dy);
  return atan2(Dy,Dx);
}

typedef struct position{
    double x; //  x and y are the real position
    double y;
    int  index_i; // index_i and index_j are the position on the matrix
    int  index_j;
}GRAPH_POSITION;  // This struct will is related to the abstract graph map that will be set

GRAPH_POSITION ** SetMap(){
    GRAPH_POSITION  ** Gmap = NULL;  //  Gmap is the matrix with the graph representation of the real map
    GRAPH_POSITION last_pos; 
    GRAPH_POSITION curr_pos;
    last_pos.x = last_pos.y = 0;

    Gmap = (GRAPH_POSITION ** )malloc(sizeof(GRAPH_POSITION)*(15));   // It will be a 15 x 13
    for(int i = 0 ; i < 15; i++){                                     // The distance between node will be 10
        Gmap[i] = (GRAPH_POSITION *)malloc(sizeof(GRAPH_POSITION)*13);
        for(int j = 0 ;  j < 13; j++){
            if(i == 0 && j == 0 ){
                curr_pos = last_pos;
                Gmap[i][j] = curr_pos;
            }else{
                curr_pos.y = last_pos.y + 10;   /// This loop is doing (0, 0 )  (0, 10 ) , ( 0 , 20) , ( 0, 30) ... in a row
                Gmap[i][j] = curr_pos;
                Gmap[i][j].index_i = i;
                Gmap[i][j].index_j = j;
                last_pos = curr_pos;
            }            
        }
        curr_pos.x += 10;  // This line change the row... ( 10 , 0 ) , (10, 10), (10, 20) , (10, 30) ...
        curr_pos.y = 0;
        last_pos.y = -10; last_pos.x = curr_pos.x;
       // printf("\nBuilded matrix\n");
    }
    return Gmap;
}
// This function find the current position of the robot on the graph_matrix
GRAPH_POSITION getCurrentNode(GRAPH_POSITION ** Map, double robot_x, double robot_y){
    GRAPH_POSITION CurrentPos;
    double ShortestDistance = 100000;
    for(int i = 0; i < 15; i ++){
        for(int j = 0 ; j < 13; j++){
            if(getDistance(robot_x, robot_y, Map[i][j].x,
            Map[i][j].y) < ShortestDistance){
                ShortestDistance = getDistance(robot_x, robot_y, Map[i][j].x, Map[i][j].y);
              printf("%lf\t",ShortestDistance);
              CurrentPos = Map[i][j];
              CurrentPos.index_i = i;
              CurrentPos.index_j = j;
            }
        }
    } 
    // the nearest node is its the current node
    return CurrentPos;
}

bool AllowedPoint(GRAPH_POSITION Position, fira_message::sim_to_ref::Frame detection){ // Verify if is a free point to move
  int robots_yellow_n =  detection.robots_yellow_size();
  bool Allowed = true;
  for(int i = 0 ; i < robots_yellow_n ; i++){
    fira_message::sim_to_ref::Robot robot_enemy = detection.robots_yellow(i);
    printf("Distance of (%d,%d) %lf", Position.index_i, Position.index_j,
    getDistance(Position.x, Position.y, robot_enemy.x(), robot_enemy.y())  );
    if(getDistance(Position.x, Position.y, robot_enemy.x(), robot_enemy.y()) < 60){ // if the yellow_root is near the point 
      printf("\t(%d,%d) is not allowed", Position.index_i, Position.index_j);
      Allowed = false;                                                              // it'isn't allowed
    }
  }
  if(Allowed)printf("\t(%d,%d) is allowed", Position.index_i, Position.index_j);  
  return Allowed;
}


// This function define the objective 
GRAPH_POSITION getNextNode(GRAPH_POSITION ** Map, GRAPH_POSITION robot, fira_message::sim_to_ref::Frame detection,
    double ball_x, double ball_y){
    //essa função varifica qual deve ser o proximo vertice que  o robo deve ir
    // vai ser o vertice com menor distancia em relação a bola e  que seja permitido
    GRAPH_POSITION NextNode;
    double ShortestDistance = 1000000000;
      if(robot.index_i == 0 || robot.index_j == 0){
        if(robot.index_i == 0 && robot.index_j == 0){
          for(int i = 0; i < 2; i++){
            for(int j = 0 ; j < 2; j++){
              if((getDistance(Map[i][j].x, Map[i][j].y,ball_x, ball_y) < ShortestDistance) && AllowedPoint(Map[i][j], detection)){
                  if(Map[i][j].x != robot.x  && Map[i][j].y != robot.y){
                  ShortestDistance = getDistance(Map[i][j].x, Map[i][j].y,ball_x, ball_y);
                  NextNode = Map[i][j];
                  }
              }
            }
          }
        }
        else if(robot.index_i == 0 && robot.index_j){
          if(robot.index_j < 12){
            for(int i  = 0 ; i < 2; i ++ ){
               for(int j  = robot.index_j - 1; j  <= robot.index_j + 1; j++){
                if((getDistance(Map[i][j].x, Map[i][j].y,ball_x, ball_y) < ShortestDistance) && AllowedPoint(Map[i][j], detection)){
                  if(Map[i][j].x != robot.x  && Map[i][j].y != robot.y){
                  ShortestDistance = getDistance(Map[i][j].x, Map[i][j].y,ball_x, ball_y);
                  NextNode = Map[i][j];
                  }
                }
               }
              }
            }else{
              for(int i  = 0 ; i < 2; i++){
               for(int j  = robot.index_j - 1; j  < robot.index_j; j++){
                if((getDistance(Map[i][j].x, Map[i][j].y,ball_x, ball_y) < ShortestDistance) && AllowedPoint(Map[i][j], detection)){
                  if(Map[i][j].x != robot.x  && Map[i][j].y != robot.y){
                  ShortestDistance = getDistance(Map[i][j].x, Map[i][j].y,ball_x, ball_y);
                  NextNode = Map[i][j];
                  }
                }
               }
              }
            }
          }
          else if(robot.index_i && robot.index_j == 0){
            if(robot.index_i < 13){
            for(int i = robot.index_i - 1; i <= robot.index_i + 1; i++){
              for(int j = robot.index_j; j <=  robot.index_j + 1; j ++){
                if((getDistance(Map[i][j].x, Map[i][j].y,ball_x, ball_y) < ShortestDistance) && AllowedPoint(Map[i][j], detection)){
                  if(Map[i][j].x != robot.x  && Map[i][j].y != robot.y){
                  ShortestDistance = getDistance(Map[i][j].x, Map[i][j].y,ball_x, ball_y);
                  NextNode = Map[i][j];
                  }
                }
              }
             }
            }else{
            for(int i = robot.index_i - 1; i < robot.index_i; i++){
              for(int j = robot.index_j; j <=  robot.index_j + 1; j ++){
                if((getDistance(Map[i][j].x, Map[i][j].y,ball_x, ball_y) < ShortestDistance) && AllowedPoint(Map[i][j], detection)){
                  if(Map[i][j].x != robot.x  && Map[i][j].y != robot.y){
                  ShortestDistance = getDistance(Map[i][j].x, Map[i][j].y,ball_x, ball_y);
                  NextNode = Map[i][j];
                  }
                }
              }
             }

            }
          }
        }
        else if(robot.index_i && robot.index_j){
            if(robot.index_i < 13){
            for(int i = robot.index_i - 1; i <= robot.index_i + 1; i++){
              for(int j = robot.index_j - 1; j <= robot.index_j+ 1; j++){
                if((getDistance(Map[i][j].x, Map[i][j].y,ball_x, ball_y) < ShortestDistance) && AllowedPoint(Map[i][j], detection)){
                  if(Map[i][j].x != robot.x  && Map[i][j].y != robot.y){
                  ShortestDistance = getDistance(Map[i][j].x, Map[i][j].y,ball_x, ball_y);
                  NextNode = Map[i][j];
                  }
                }
              }
            }
        }
            else{
              for(int i = robot.index_i - 1; i < robot.index_i; i++){
              for(int j = robot.index_j - 1; j <= robot.index_j+ 1; j++){
                if((getDistance(Map[i][j].x, Map[i][j].y,ball_x, ball_y) < ShortestDistance) && AllowedPoint(Map[i][j], detection)){
                  if(Map[i][j].x != robot.x  && Map[i][j].y != robot.y){
                  ShortestDistance = getDistance(Map[i][j].x, Map[i][j].y,ball_x, ball_y);
                  NextNode = Map[i][j];
                  }
                }
              }
            }
            }
          }
      return NextNode;
  }





double x,y,ang;

   

Objective defineObjective(fira_message::sim_to_ref::Robot robot, fira_message::sim_to_ref::Ball ball,
                         fira_message::sim_to_ref::Frame detection)
{   
    //Implementar a estratégia aqui
    //Deve retornar o objetivo do robô
    GRAPH_POSITION CurrNode;
    GRAPH_POSITION NextNode;
    GRAPH_POSITION ** Map = NULL;
    Map = SetMap();  // Gera o mapa na forma de grafo
    double last_position_x;
    double last_position_y;
    CurrNode = getCurrentNode(Map, robot.x(), robot.y());  // pega o vertice atual do robo azul
    GRAPH_POSITION BallNode = getCurrentNode(Map, ball.x(), ball.y());
    if(getDistance(robot.x(),robot.y(),ball.x(),ball.y()) < 7.01){
      // Se a bola estiver em uma distancia menor do que 6.01 , então o objetivo vira o ponto (155,65), ponto esse que fica
      //exatamente na metade do gol amarelo
      x = 155;
      y = 65;
       robot.set_vx(0.05);
        robot.set_vy(0.05);
      ang = getAngle(robot.x(),robot.y(),x,y); // aqui pega o angulo que ele deve virar em relação ao gol
      }else{
      
       if(robot.robot_id() == 1){ // se o robo for o numero 1, ele será o goleiro que só se move nessas margens que seguem
        if(ball.x() > 10 && ball.x() < 25){
          x = ball.x();
        }else{
          x = 20;
        }
        if(ball.y() > 30 && ball.y() < 100){
          y = ball.y();
        }else{
          y  = 50;
        }
        // dessa forma que foi escrita, o goleiro consegue "acompanhar" o movimento da bola sem que saia dos limites de 
        // sua posição definida
        ang = getAngle(ball.x(), ball.y(), robot.x(), robot.y()); 
      }else if(CurrNode.index_i != NextNode.index_i && CurrNode.index_j !=  NextNode.index_j
         && CurrNode.index_i != BallNode.index_i && CurrNode.index_j != BallNode.index_j){
        
        NextNode = getNextNode(Map,CurrNode,detection, ball.x(), ball.y()); // essa função pega o  proximo vértice que 
        // o robo deve ir
        x =  NextNode.x;
        y = NextNode.y;
        y = ball.y();
       x = ball.x();

      }else{
       
        x = ball.x();
        y = ball.y();
      }
      
      last_position_x = x;
      last_position_y = y;
      }
    for(int i = 0 ;  i  < 15; i ++ ){
      free(Map[i]);
    }
    return Objective(x, y, ang);
}

int main(int argc, char *argv[]){
    (void)argc;
    (void)argv;
    //define your team color here
    bool my_robots_are_yellow = false;
    
    // the ip address need to be in the range 224.0.0.0 through 239.255.255.255
    RoboCupSSLClient *visionClient = new RoboCupSSLClient("224.5.23.2", 10002);
    visionClient->open(false);


    GrSim_Client *commandClient = new GrSim_Client();
    

    fira_message::sim_to_ref::Environment packet;

    while(true) {
        if (visionClient->receive(packet)) {
            printf("-----Received Wrapper Packet---------------------------------------------\n");
            //see if the packet contains a robot detection frame:
            if (packet.has_frame()) {
                fira_message::sim_to_ref::Frame detection = packet.frame();



                int robots_blue_n =  detection.robots_blue_size();
                int robots_yellow_n =  detection.robots_yellow_size();
                double width,length;
                width = 1.3/2.0;
                length = 1.7/2.0;

                //Ball info:

                fira_message::sim_to_ref::Ball ball = detection.ball();
                ball.set_x((length+ball.x())*100);
                ball.set_y((width+ball.y())*100);
                printf("-Ball:  POS=<%9.2f,%9.2f> \n",ball.x(),ball.y());



                //Blue robot info:
                for (int i = 0; i < robots_blue_n; i++) {
                    fira_message::sim_to_ref::Robot robot = detection.robots_blue(i);
                    robot.set_x((length+robot.x())*100);//convertendo para centimetros
                    robot.set_y((width+robot.y())*100);
                    robot.set_orientation(to180range(robot.orientation()));
                    printf("-Robot(B) (%2d/%2d): ",i+1, robots_blue_n);
                    printRobotInfo(robot);
                    if(i==0 || i == 1){
                        Objective o = defineObjective(robot,ball, detection);
                        PID(robot,o,i,commandClient);
                    }
                }

                //Yellow robot info:
                /*for (int i = 0; i < robots_yellow_n; i++) {
                    fira_message::sim_to_ref::Robot robot = detection.robots_yellow(i);
                    printf("-Robot(Y) (%2d/%2d): ",i+1, robots_yellow_n);
                    printRobotInfo(robot);
                }*/

            }

            //see if packet contains geometry data:
            /*if (packet.has_field()){
                printf("-[Geometry Data]-------\n");

                const fira_message::sim_to_ref::Field & field = packet.field();
                printf("Field Dimensions:\n");
                printf("  -field_length=%f (mm)\n",field.length());
                printf("  -field_width=%f (mm)\n",field.width());
                printf("  -goal_width=%f (mm)\n",field.goal_width());
                printf("  -goal_depth=%f (mm)\n",field.goal_depth());



            }*/
        }
    }

    return 0;
}


