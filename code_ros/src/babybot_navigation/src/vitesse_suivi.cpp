#include <stdio.h>
#include <cmath>
#include <../../opt/ros/indigo/include/geometry_msgs/Twist.h>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <std_msgs/MultiArrayLayout.h>
#include "std_msgs/Int32.h"
#include <std_msgs/MultiArrayDimension.h>

std_msgs::Int32 test;


ros::Publisher pub; //declaration publisher
std_msgs::MultiArrayDimension msg_dim; //variable de dimension du tableau
std_msgs::Float32MultiArray msg; //varriable du tableau qui publie les vitesses des moteurs
geometry_msgs::Twist val; // variable a recevoir

void callback( const geometry_msgs::Twist& val)
{
  //recuperation des donnees lues sur le subscriber
  float vx = val.linear.x; //vx
  float vy = val.linear.y; //vy
  float w = val.angular.z; //w
 
  msg.data.clear();
  msg.data.push_back(vx);
  msg.data.push_back(vy);
  msg.data.push_back(w);
  
  if(test.data == 3){
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
  ros::init(argc, argv, "vitesse_suivi");

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
  ros::Subscriber sub =  n.subscribe("cmd_vel", 1, callback);
  
  ros::spin();
}

