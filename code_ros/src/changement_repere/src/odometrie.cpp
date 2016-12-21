#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16MultiArray.h>
#include <iostream>
#include <math.h>
#include <../../opt/ros/indigo/include/tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#define RAYON 0.05
#define LONGUEUR 0.32
#define LARGEUR 0.19
#define RAPPORT (15.0/6000.0)*1.1 //1.15 est un rapport d'erreur
#define PI 3.14159265358979323846

//Réalise la liaison entre "/odom" et "/base_link" pour réaliser l'odométrie du robot

ros::Publisher publi;
tf::StampedTransform transform_lue;
std_msgs::Int16MultiArray donnees;
std_msgs::Float32MultiArray vitesses;
std_msgs::MultiArrayDimension vitesses_dim;


float x=0, y=0, teta=0, cos2, sin2, relais;
float enc1, enc2, enc3, enc4;
float vy, vx, w, w1, w2, w3, w4;
float x_angle, y_angle, x_envoi, y_envoi, angle_envoi, angle, vx_ajout, vy_ajout;

void chatterCallback(std_msgs::Int16MultiArray::ConstPtr valeurs){
  
  ROS_INFO("--------------");
  
  donnees = *valeurs;

  //Modèle géométrique direct pour récupérer Vx, Vy, W du robot à partir des encodeurs moteurs
  if(valeurs->data[0] < 0){
    enc1 = -(float)(valeurs->data[0]);
    w1 = -enc1*(RAPPORT);
  }
  else{
    enc1 = (float)(valeurs->data[0]);
    w1 = enc1*(RAPPORT);
  }
  
  
  if(valeurs->data[1] < 0){ 
    enc2 = -(float)(valeurs->data[1]);
    w2 = -enc2*(RAPPORT);
  }
  else{
    enc2 = (float)(valeurs->data[1]);
    w2 = enc2*(RAPPORT);
  }
  
  
  if(valeurs->data[2] < 0){
    enc3 = -(float)(valeurs->data[2]);
    w3 = -enc3*(RAPPORT);
  }
  else{
    enc3 = (float)(valeurs->data[2]);
    w3 = enc3*(RAPPORT);
  }
  
  
  if(valeurs->data[3] < 0){
    enc4 = -(float)(valeurs->data[3]);
    w4 = -enc4*(RAPPORT);
  }
  else{
    enc4 = (float)(valeurs->data[3]);
    w4 = enc4*(RAPPORT);
  }
  
  vx = (w1+ w2 + w3 + w4)*( RAYON / 4);
  vy = -(-w4 + w1 + w3 - w2)*( RAYON / 4);
  w = (-w4 + w1 - w3 + w2)*(RAYON/(4*(LONGUEUR + LARGEUR)));
  
  //Calcul de x, y et teta de l'odometrie en fonction de Vx, Vy, et W
  //On calcule les coefficients "52.5" et "50" en faisant des tests avec le robot pour que l'odométrie suive bien les mouvements réels du robot.
  angle = w/52.5;
  if(angle > PI){
    angle = angle - PI;
    angle = - angle;
  }else if(angle < -PI){
    angle = angle + PI;
    angle = -angle;
  }
  vx_ajout = vx/50;
  vy_ajout = vy/50;
  
  //On ajoute les vraies déplacements en x et en y en fonction de la rotation instantannée du robot
  x_angle = (vx_ajout*cos(angle) + vy_ajout*sin(angle));
  y_angle = -vx_ajout*sin(angle) + vy_ajout*cos(angle);
  
  //On ajoute les déplacements et l'angle à ceux de la transformation avant mouvement entre "/odom" et "/base_link"
  x_envoi = x + x_angle*cos(teta) - y_angle*sin(teta);
  y_envoi = y + x_angle*sin(teta) + y_angle*cos(teta);
  angle_envoi = teta + angle;
  /*ROS_INFO("vx : %f, vy : %f, w : %f\n", vx, vy, w);
  ROS_INFO("vx_ajout : %f, vy_ajout : %f\n", vx_ajout, vy_ajout);
  ROS_INFO("x_angle : %f, y_angle : %f\n", x_angle, y_angle);
  ROS_INFO("x_envoi : %f, y_envoi : %f, angle_envoi:%f\n", x_envoi, y_envoi, angle_envoi, angle);*/
  
  //publication sur publi
  vitesses.data.clear();
  vitesses.data.push_back(vx);
  vitesses.data.push_back(vy);
  vitesses.data.push_back(w);
  publi.publish(vitesses);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometrie");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe<std_msgs::Int16MultiArray>("valeurs_encodeurs", 1, chatterCallback);
  ros::Rate loop_rate(20);
  vitesses_dim.label = "odometrie";
  vitesses_dim.size = 3;
  vitesses.layout.dim.clear();
  vitesses.layout.dim.push_back(vitesses_dim);
  publi = node.advertise<std_msgs::Float32MultiArray>("vit_encodeurs",1);
  tf::TransformBroadcaster br;
  tf::TransformListener li;
  tf::Transform transform;
  while(ros::ok()){
    //On regarde la transformation entre "/odom" et "/base_link" pour récupérer les déplacements et l'angle déjà effectués
    try{
      li.lookupTransform("/odom", "/base_link", ros::Time(0), transform_lue);
    }
    catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
// 	ros::Duration(0.5).sleep();
// 	continue;
    }
    x = transform_lue.getOrigin().x();
    y = transform_lue.getOrigin().y();
    teta = 2*atan2(transform_lue.getRotation().getZ(),transform_lue.getRotation().getW());
    
    
    ros::spinOnce();
    
    //Envoi de la nouvelle transformation entre "/odom" et "/base_link"
    cos2 = cos(angle_envoi/2);
    sin2 = sin(angle_envoi/2);
    loop_rate.sleep();
    transform.setOrigin(tf::Vector3(x_envoi, y_envoi, 0.0));
    transform.setRotation(tf::Quaternion(0.0, 0.0,sin2,cos2)); //On effectue la rotation de la frame en fonction de l'angle de la caméra
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link")); //Création de la nouvelle frame**

    }
  return 0;
}
      
