#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#define PI 3.14159265358979323846

//Fonction qui crée la liaison entre "/base_link" et "/camera_link", la frame de la kinect ajustée avec son angle à l'horizontale

int main(int argc, char** argv){
  ros::init(argc, argv, "cree_frame");
  ros::NodeHandle node;
  float alpha = 3; //Angle de la caméra par rapport au plan horizontal
  float cos_alpha2 = cos(alpha*PI/360);
  float sin_alpha2 = -sin(alpha*PI/360);
  
  tf::TransformBroadcaster br;
  tf::Transform transform;
  
  ros::Rate rate(10.0);
  while(ros::ok()){
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform.setRotation(tf::Quaternion(0.0, sin_alpha2, 0.0, cos_alpha2)); //On effectue la rotation de la frame en fonction de l'angle de la caméra
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera_link"));
    rate.sleep();
  }
  return 0;
}