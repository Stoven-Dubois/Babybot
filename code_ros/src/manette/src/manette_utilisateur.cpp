#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

//Permet à l'utilisateur de choisir le mode du fonctionnement du robot à partir de la manette

ros::Publisher publi;
sensor_msgs::Joy new_joy;
std_msgs::Float32MultiArray donnees;
std_msgs::Int32 choix;

void chatterCallback(sensor_msgs::Joy::ConstPtr joy){
  new_joy = *joy;
  float palier=0.1;
  int test = 0;
  donnees.data.clear();
  for(int i=0;i<4;i++){
    donnees.data.push_back(new_joy.buttons[i]); //0 : a, 1 : b, x : 2, y : 3
  }
  for(int i=0;i<4;i++){
    if(donnees.data[i]!=0){
      test++;
      choix.data = i+1; //1 : a, 2 : b, x : 3, y : 4
    }
  }
  if(test !=1){
    choix.data = 0;
  }else{
    if(choix.data == 2){
      choix.data = -1;
    }
    if(choix.data == 3){
      choix.data = 2;
    }
    if(choix.data == 4){
      choix.data = 3;
    }
  }
  ROS_INFO("%d\n",choix.data);
  publi.publish(choix);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "manette_utilisateur");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe<sensor_msgs::Joy>("/joy", 1, chatterCallback);
  publi = node.advertise<std_msgs::Int32>("/choix", 1);
  ros::Rate loop_rate(10);
  ros::spin();
  return 0;
}
      
