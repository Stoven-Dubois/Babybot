#include <stdio.h>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#define LONGUEUR 0.32
#define LARGEUR 0.19
#define RAYON 0.05
#define PI 3.14159265358979323846
#define WMAX 1
#define COEF_JOY 3.0 //coef lié a l'ecrasement des joystick pour recuperer une valeur entre -1 et 1
#define ZONE_MORTE 0.1

ros::Publisher pub; //declaration du publisher
std_msgs::MultiArrayDimension msg_dim; //variable de dimension du tableau
std_msgs::Float32MultiArray msg; //varriable du tableau
std_msgs::Int32 test;

void callback( const std_msgs::Int16MultiArray& val)
{
  //recuperation des donnees lues sur le subscriber
  float joy_X1 = (float)val.data[0]*COEF_JOY/100; //xa
  float joy_Y1 = (float)val.data[1]*COEF_JOY/100; //ya
  float joy_X2 = (float)val.data[2]*COEF_JOY/100; //xb
  float joy_Y2 = (float)val.data[3]*COEF_JOY/100; //yb
   
  
  if(fabs(joy_X1) < ZONE_MORTE)
  {
  	joy_X1 = 0.0;
  }
  if(fabs(joy_Y1) < ZONE_MORTE)
  {
  	joy_Y1 = 0.0;
  }
  if(fabs(joy_X2) < ZONE_MORTE)
  {
  	joy_X2 = 0.0;
  }
  if(fabs(joy_Y2) < ZONE_MORTE)
  {
  	joy_Y2 = 0.0;
  }
 
  
  float va = sqrt(joy_X1*joy_X1 + joy_Y1*joy_Y1);
  float vb = sqrt(joy_X2*joy_X2 + joy_Y2*joy_Y2);
    
  float aj1 = atan2(joy_X1,joy_Y1);
  float aj2 = atan2(joy_X2,joy_Y2);

  float winter = (WMAX*(aj1-aj2))/PI;
  float wtot = winter*(va+vb);
  
  float vx_tot = (joy_X1 + joy_X2)/2;
  float vy_tot = (joy_Y1 + joy_Y2)/2;
  
 
  if(fabs(vx_tot) < 0.1){vx_tot = 0.0;}
  if(fabs(vy_tot) < 0.1){vy_tot = 0.0;}
  
  
  msg.data.clear();
  msg.data.push_back(vx_tot);
  msg.data.push_back(vy_tot);
  msg.data.push_back(wtot);
    
    if(test.data == 2)
  {
    //publication sur le publisher
    pub.publish(msg);
  }
}

void callback_choix(std_msgs::Int32::ConstPtr choix){
  std_msgs::Int32 valeur_choix = *choix;
  if(valeur_choix.data !=0){
    test.data = valeur_choix.data;
  }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "calcul_vitesse");

  ros::NodeHandle n; //declaration de nodehandle  
  //initilisation des dimensions du tableau
  msg_dim.label = "vitesses";
  msg_dim.size = 3;
  
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);
  
  //initalisation du publisher
  pub = n.advertise<std_msgs::Float32MultiArray>("valeurs_triplet", 1);
  
    //sub de la commande mannette
  ros::Subscriber commande = n.subscribe("/choix",1, callback_choix);
  ros::Subscriber sub =  n.subscribe("valeur_joy", 1, callback);
  

  ros::spin();
  return 0;
}
  
  
