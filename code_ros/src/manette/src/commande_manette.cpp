#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>


//Permet de commander le robot avec les joysticks de la manette
ros::Publisher publi;
sensor_msgs::Joy new_joy;
std_msgs::Float32MultiArray donnees;
std_msgs::Int32 test;

ros::Time lastcall;

void chatterCallback(sensor_msgs::Joy::ConstPtr joy){
  new_joy = *joy;
  float palier = 0.1, coupure = 0.2, angle;
  int compteur;
  donnees.data.clear();
  donnees.data.push_back(new_joy.axes[1]); //x
  
  donnees.data.push_back(new_joy.axes[0]); //y
  angle = new_joy.axes[3];
  if(angle > 3.14){
    angle = angle- 3.14;
  }
  if(angle<-3.14){
    angle = angle + 3.14;
  }
  donnees.data.push_back(angle);  //theta
  for(int i=0;i<3;i++){
    if(donnees.data[i]<coupure && donnees.data[i]>(-coupure)){
      donnees.data[i] = 0.0;
    }
  }
  
  //Fonctionnement par palier des différentes vitesses
  for(int i=0;i<1;i++){
    compteur = 0;
    for(float j=1.0/palier;j>0.0;j--){
      if(donnees.data[i]<(j*palier) && donnees.data[i]>((-j)*palier)){
	compteur++;
      }
    }
    if(donnees.data[i]>0){
      donnees.data[i] = 1 - compteur*palier;
    }else{
      donnees.data[i] = -(1 - compteur*palier);
    }
  }
  donnees.data[2] = 2*donnees.data[2];
  ROS_INFO("%f %f %f\n",donnees.data[0], donnees.data[1], donnees.data[2]);
  
  lastcall = ros::Time::now();
}

void testCallback(std_msgs::Int32::ConstPtr choix){
  std_msgs::Int32 valeur_choix = *choix;
  if(valeur_choix.data !=0){
    test.data = valeur_choix.data;
  }
}
  

int main(int argc, char** argv){
  ros::init(argc, argv, "commande_manette");
  ros::NodeHandle node;
  lastcall = ros::Time::now();
  ros::Subscriber sub = node.subscribe<sensor_msgs::Joy>("/joy", 1, chatterCallback);
  ros::Subscriber sub2 = node.subscribe<std_msgs::Int32>("/choix",1,testCallback);
  publi = node.advertise<std_msgs::Float32MultiArray>("/valeurs_triplet", 1);
  ros::Rate loop_rate(10);
  ros::Duration time_diff;
  while(ros::ok()){
    if(test.data == 1){
      
      time_diff = ros::Time::now() - lastcall;
      /*if(time_diff.sec > 0.5){
      //publication sur valeur_moteur de 0 pour un arret d'urgence
	donnees.data.clear();
	for(int i=0;i<3;i++){
	  donnees.data.push_back(0);
	}
      }*/
      publi.publish(donnees);
    }
    if(test.data == -1){
       //publication sur valeur_moteur de 0 pour un arret d'urgence
	donnees.data.clear();
	for(int i=0;i<3;i++){
	  donnees.data.push_back(0);
	}
	publi.publish(donnees);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  //Gestion non-envoi manette
  
  return 0;
}
      
