#include "BMSerial.h"
#include "RoboClaw.h"
#include <Servo.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <Arduino.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <ros/time.h>

ros::NodeHandle n; //declaration de nodehandle
std_msgs::Int16MultiArray msg;
std_msgs::Int16MultiArray enc;
std_msgs::Int32MultiArray cam_msg;
ros::Publisher pub("valeur_joy", &msg); //declaration du pub des joysticks
ros::Publisher encodeur("valeurs_encodeurs", &enc); //declaration du pub des encodeurs
ros::Publisher pub_cam( "cam_data", &cam_msg);  //declaration du pub de la position de la camera
  
//Roboclaw Address
#define address 0x80
//Velocity PID coefficients
#define Kp 1.0
#define Ki 0.0
#define Kd 0.0
#define qpps 1600
#define r 0.05
#define coef_max 127 // les fonctions backward et forward fonctionnent entre 0 (stop) et 127 (vitesse max)
#define tours_max 12 // nb de tours par secondes a la vitesse 127 a vide en 5s
#define t 5 // nb de secondes utilisees pour la mesure de tours_max
#define Vroue_max 0.75 // le calcul est  V= 2*PI*r*tours_max/t
#define camRot 800 // test

// Capteur Joystick
double calX1, calY1, rawX1, rawY1;
double calX2, calY2, rawX2, rawY2;
// Variable filtre
float xv[3][4];
float yv[3][4];
float previous[4];
float p[4];
// Coordonées x et y de la balle sur l'écran
int camX, OldcamX, camY, distance, av,ar,g,d;
// variables relatives au servo
boolean sens;
Servo servoCam;
int tps_avant = 0;  
boolean detect = false;
float sensibilite = 1.3;
int ampli_rot_cam =  1;
int ampli_tran_cam = 10;
int ampli_rot_joystick = 8;
int posCam = 90;
int oldCam = 90;
float posCamtemp = 90.0;
unsigned long cam_timer;
int seuil_rotation = 10;
// Angle de rotation de la base mobile
int alpha = 0;
// zone morte des joysticks
int mini = 0.10;
float angle = 0.0;
// Vitesse de chaque moteur
float vit1, vit2, vit3, vit4;
int vf1, vf2, vf3, vf4;
//temps d'execution
unsigned long start_time;
unsigned long end_time;

//Setup communcaitions with roboclaw. Use pins 10 and 11 with 10ms timeout
RoboClaw roboclaw(17,16,10000);
RoboClaw roboclaw2(15,14,10000);
std_msgs::MultiArrayDimension msg_dim; //variable de dimension du tableau
void callback( const std_msgs::Float32MultiArray& val){
  vit1 = val.data[0];
  vit2 = val.data[1];
  vit3 = val.data[2];
  vit4 = val.data[3];
}
ros::Subscriber<std_msgs::Float32MultiArray> sub("valeur_moteur", &callback, 1 );//declaration du sub

void cb_servo( const std_msgs::Int32MultiArray& val){
  
   OldcamX = camX;
   camX =  val.data[0];
   camY =  val.data[1];  
   distance =  val.data[2];
   
   //if(camX != 0 || camY != 0 || distance != 0){
     if(camX == 1){
     angle =  val.data[2];
     detect = true;
     tps_avant = micros();
     if(oldCam < angle){
      sens = true; 
     }
     else{
      sens = false; 
     }
   }
   else{
     if(micros() - tps_avant > 300){
       detect = false;
     }
   }
   
}

ros::Subscriber<std_msgs::Int32MultiArray> sub_servo("Servo", &cb_servo, 1 );

void filterloop( float p[4] ){
     for( int i = 0; i<4 ; i++){
         //filtre maison
          p[i] = (0.09063*p[i] + 0.8187*(p[i]+previous[i])/2);
          previous[i] = p[i];       
     }            
}

void setup() {
  vit1 = 0; 
  vit2 = 0; 
  vit3 = 0; 
  vit4 = 0;
  for( int i = 0; i<4 ; i++){
    previous[i] = 0.0;       
   }     
  
  n.initNode();
 
  n.advertise(pub);
  n.advertise(encodeur);
  n.advertise(pub_cam);
  n.subscribe(sub);
  n.subscribe(sub_servo);
  
  //Serial.begin(9600); // ne pas decommenter, probleme de communication
  
  roboclaw.begin(38400);
  roboclaw2.begin(38400);
  
  //initialisation du tableau des joyticks
  msg.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  msg.layout.dim[0].label = "msg";
  msg.layout.dim[0].size = 4;
  msg.layout.dim[0].stride = 1*4;
  msg.layout.data_offset = 0;
    
  msg.data = (int *)malloc(sizeof(int)*4);
  msg.data_length = 4;
  
  //initialisation du tableau des encodeurs
  enc.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  enc.layout.dim[0].label = "enc";
  enc.layout.dim[0].size = 4;
  enc.layout.dim[0].stride = 1*4;
  enc.layout.data_offset = 0;
  
  enc.data = (int *)malloc(sizeof(int)*4);
  enc.data_length = 4;
  
  // une led
  pinMode(50, OUTPUT);
  digitalWrite( 50, HIGH );
 
  // initialisation du servo moteur
  servoCam.attach(9);
  servoCam.write(90);
  sens = true;
  cam_msg.data_length = 3;
  
  //Set PID Coefficients
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2VelocityPID(address,Kd,Kp,Ki,qpps);  
  roboclaw2.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw2.SetM2VelocityPID(address,Kd,Kp,Ki,qpps);  
 
  pinMode( A0, INPUT );
  pinMode( A1, INPUT );
  pinMode( A2, INPUT );
  pinMode( A3, INPUT );
  
  //ATTENTION AU SENS DU MONTAGE DES JOYSTICKS
  calX1 = analogRead( A0 );
  calY1 = analogRead( A1 );
  calX2 = analogRead( A2 );
  calY2 = analogRead( A3 );
   
}
void loop(){
  
  start_time = millis();
  
  //on envoie sur ros les valeurs des joys comprisent entre -1 et 1
  rawX1 = -(analogRead( A0 ) - calX1);
  rawY1 = -(analogRead( A1 ) - calY1);  
  rawX2 = (analogRead( A2 ) - calX2);
  rawY2 = (analogRead( A3 ) - calY2);
  
  p[0] = rawX1/calX1;
  p[1] = rawY1/calY1;
  p[2] = rawX2/calX2;
  p[3] = rawY2/calY2;
  
  filterloop( p );
  
  msg.data[0] = p[0]*100;
  msg.data[1] = p[1]*100;
  msg.data[2] = p[2]*100;
  msg.data[3] = p[3]*100;
    
  //publication sur le publisher des joysticks
  pub.publish(&msg);
  
 

  //test de vitesse
  
  /*int32_t speeed;
   int oo,uu;
  bool val;
  uint8_t stat;
  for(oo=0; oo<500;oo++){
  roboclaw.BackwardM1(address, 0);
  speeed = roboclaw.ReadSpeedM1(address, &stat, &val);
   Serial.print("speed : ");
  //if(stat){
   
    Serial.print(speeed);
  //}
  Serial.println();
  delay(10);
  }
  for(uu=0; uu<300;uu++){
  //roboclaw.BackwardM2(address, 0);
  roboclaw.BackwardM1(address, 0);
  delay(10);
  }*/
  
  //envoi les vitesses aux roboclaws en les castans en int, les moteurs de la roboclaw 1 sont a l envers sur le montage actuel
  if(vit1>=0){
    int vf1 = (r*coef_max*vit1)/Vroue_max;
    roboclaw2.ForwardM2(address, vf1);
  }
  else{
     int vf1 = (-r*coef_max*vit1)/Vroue_max;
     roboclaw2.BackwardM2(address, vf1);
  }
   if(vit2<0){
    int vf2 = (-r*coef_max*vit2)/Vroue_max;
    roboclaw.ForwardM1(address, vf2);
  }
  else{
    int vf2 = (r*coef_max*vit2)/Vroue_max;
     roboclaw.BackwardM1(address, vf2);
  }
   if(vit3<0){
   int vf3 = (-r*coef_max*vit3)/Vroue_max;
    roboclaw.ForwardM2(address, vf3);
  }
  else{
     int vf3 = (r*coef_max*vit3)/Vroue_max;
     roboclaw.BackwardM2(address, vf3);
  }
   if(vit4>=0){
    int vf4 = (r*coef_max*vit4)/Vroue_max;
    roboclaw2.ForwardM1(address, vf4);
  }
  else{
     int vf4 = (-r*coef_max*vit4)/Vroue_max;
     roboclaw2.BackwardM1(address, vf4);
  }
  
  uint8_t status1;
  bool valid1;
  int32_t speed;
  speed = roboclaw2.ReadSpeedM2(address, &status1, &valid1);
  if(valid1){
    enc.data[0] = speed;
  }
  speed = roboclaw.ReadSpeedM1(address, &status1, &valid1);
  if(valid1){
    enc.data[1] = speed;
  }
  speed = roboclaw.ReadSpeedM2(address, &status1, &valid1);
  if(valid1){
    enc.data[2] = speed;
  }
  speed = roboclaw2.ReadSpeedM1(address, &status1, &valid1);
  if(valid1){
    enc.data[3] = speed;
  }
  
  //modification de derniere minute a cause de l'encodeur 2
    int vf1 = ((r*coef_max*vit1)/Vroue_max) * (4000/256) * (4000/1270);
    enc.data[0] =  vf1;
    int vf2 = ((r*coef_max*vit2)/Vroue_max) * (4000/256) * (4000/1270);
    enc.data[1] =  vf2;
    int vf3 = ((r*coef_max*vit3)/Vroue_max) * (4000/256) * (4000/1270);
    enc.data[2] =  vf3;
    int vf4 = ((r*coef_max*vit4)/Vroue_max) * (4000/256) * (4000/1270);
    enc.data[3] =  vf4;

  //publication sur le publisher des encodeurs
  encodeur.publish(&enc);
  
  // mettre ici la separation du mode suivi
 
    
     /*if(detect){ // utilisation camera
        alpha = (90-posCam);
        if( alpha > seuil_rotation ){
           alpha -= seuil_rotation;
        }
        else{
          if( alpha < -seuil_rotation ){
            alpha += seuil_rotation;
          }
          else{
            alpha = 0;
          }
        }
        alpha = alpha*ampli_rot_cam;
        
        cam_msg.data[1] = 50 - distance;
        cam_msg.data[2] = posCam;
        pub_cam.publish(&cam_msg);
        
     }*/
    
  
  //fin du code
  n.spinOnce();
  end_time = millis();
  
  // Mode Camera
    
//      if(detect){
//        if (camX < 0){
//          if (posCamtemp >= 180){
//            posCamtemp = 180;
//          }else{
//            posCamtemp -= camX/camRot;
//            //posCamtemp -= 0.2*abs(camX)/(camX);
//            sens = true;
//          }
//        }
//        else{
//          if (posCamtemp <= 0){
//            posCamtemp = 0;
//          }else{
//            posCamtemp -= camX/camRot;
//            //posCamtemp -= 0.2*abs(camX)/(camX);
//            sens = false;
//          }
//        }
//      }else{
//      if (sens == true){
//        if (posCamtemp >= 180){
//          posCamtemp = 180;
//          sens = false;
//        }else{
//          posCamtemp += 0.2;
//        }
//      }else{
//        if (posCamtemp <= 0){
//          posCamtemp = 0;
//          sens = true;
//        }else{
//          posCamtemp -= 0.2;
//        }
//      }
//     }
    
    
    if(detect){
     posCamtemp = angle;
     if(angle > 180) posCamtemp = 180;
     if(angle < 0) posCamtemp = 0;
     
    }
    else{
      if (sens == true){
        if (posCamtemp >= 180){
          posCamtemp = 180;
          sens = false;
        }else{
          posCamtemp += 0.25;
        }
      }else{
        if (posCamtemp <= 0){
          posCamtemp = 0;
          sens = true;
        }else{
          posCamtemp -= 0.25;
        }
      }
     }
     posCam = (int)(abs(posCamtemp-9*oldCam/10));
     oldCam = posCam;
     servoCam.write(posCam);
     cam_msg.data[2] = posCam;
     pub_cam.publish(&cam_msg);
  /*Serial.print("start time = ");
  Serial.print(start_time);
  Serial.print(" end_time = ");
  Serial.print(end_time);
  Serial.print("total : ");
  Serial.println(end_time-start_time);*/
  /*if(end_time-start_time < 50){
    delay(50 + start_time - end_time);
  }*/
}
