#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>

//Ajoute un champ pour l'intensité au signal de la kinect

ros::Publisher publi;
sensor_msgs::LaserScan new_scan;

void chatterCallback(sensor_msgs::LaserScan::ConstPtr scan){
  new_scan = *scan;
  int nb_mesures = ((new_scan.angle_max - new_scan.angle_min)/new_scan.angle_increment)+1;
  int i;
  new_scan.intensities.resize(nb_mesures);
  for(i=0;i<nb_mesures;i++){
    new_scan.intensities[i] = 0.0	; //On initialise le champ intensities du laser issu de la kinect à 0
  }
  //new_scan.header.frame_id = "nv_camera_link"; 
  publi.publish(new_scan);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "ajout_intensite");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe<sensor_msgs::LaserScan>("/nveau_scan", 1, chatterCallback);
  publi = node.advertise<sensor_msgs::LaserScan>("/scan_intensite", 1);
  ros::Rate loop_rate(10);
  ros::spin();
  return 0;
}
      