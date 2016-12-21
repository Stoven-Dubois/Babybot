#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <math.h>
#define PI 3.14159265358979323846

//Rectifie les valeurs du signal de la kinect en fonction de son angle par rapport à l'horizontale

ros::Publisher publi;
sensor_msgs::LaserScan new_scan;

void chatterCallback(sensor_msgs::LaserScan::ConstPtr scan){
  new_scan = *scan;
  int nb_mesures = ((new_scan.angle_max - new_scan.angle_min)/new_scan.angle_increment)+1;
  int i;
  float alpha = 3;	//angle de la caméra par rapport au plan horizontal
  float cosAlpha = cos(alpha*PI/360);
  for(i=0;i<nb_mesures;i++){
    new_scan.ranges[i] = new_scan.ranges[i]*cosAlpha; //On met les valeurs du laser issu de la kinect à la bonne valeur en fonction de l'angle de la caméra
  }
  publi.publish(new_scan);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "changement_repere_node");
  ros::NodeHandle node;
  publi = node.advertise<sensor_msgs::LaserScan>("nveau_scan", 1000);
  ros::Subscriber sub = node.subscribe("kinect_scan", 1, chatterCallback);
  ros::Rate loop_rate(10);
  ros::spin();
  return 0;
}
      
