#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32MultiArray.h>
#define PI 3.14159265358979323846

//Programme qui écrit la relation entre "/base_link" et la frame de la caméra

float angle = 0;//Angle de rotation de la caméra par rapport au plan horizontal
float largeur=50, longueur=84; //Dimensions du robot

void callback(std_msgs::Int32MultiArray::ConstPtr angleRecu){
  angle = (float) angleRecu->data[2];
  angle = angle - 90;
  angle = angle*PI/180;
  ROS_INFO("angle\n");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "cree_frame");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/cam_data", 1, callback);
  float cos_angle2;
  float sin_angle2;
  
  tf::TransformBroadcaster br;
  tf::Transform transform1, transform2, transform3, transform4, transform5, transform6; //Transformations appliquées pour créer la frame de la caméra
  
  ros::Rate rate(50.0);
  while(ros::ok()){
    cos_angle2 = cos(angle/2.0);
    sin_angle2 = sin(angle/2.0);
    
    //On passe "/base_link" à "/head_camera" en changeant l'origine et en faisant tourner le plan (x, y) selon l'angle de la caméra
    transform1.setOrigin(tf::Vector3(longueur/200, largeur/200, 119.0/100));
    transform1.setRotation(tf::Quaternion(0.0, 0.0, sin_angle2, cos_angle2)); //On effectue la rotation de la frame en fonction de l'angle de la caméra
    
    //Transformations successives pour arriver au bon sens des axes pour la lecture du code
    transform2.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform2.setRotation(tf::Quaternion(0.0, sin(PI/4), 0.0, cos(PI/4)));
    transform3.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform3.setRotation(tf::Quaternion(sin(-PI/2), 0.0, 0.0, cos(PI/2)));
    transform4.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform4.setRotation(tf::Quaternion(0.0, 0.0, sin(PI/2), cos(PI/2)));
    transform5.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform5.setRotation(tf::Quaternion(sin(PI/2), 0.0, 0.0, cos(PI/2)));
    transform6.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform6.setRotation(tf::Quaternion(0.0, 0.0, sin(-PI/4), cos(-PI/4)));
    
   //On applique les transformations
    transform1 *=transform2;
    transform1 *=transform3;
    //transform1 *=transform4;
    transform1 *=transform5;
    transform1 *=transform6;	
    //On envoit la transformation
    br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "base_link", "head_camera"));
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}