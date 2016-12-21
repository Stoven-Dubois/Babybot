#include <stdio.h>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#define LONGUEUR 0.32
#define LARGEUR 0.19
#define RAYON 0.05
#define VMAX 0.6
#define VMAXMOT 10
#define VMIN 0.1
#define WMIN 0.2
#define WMAX 0.7
#define KACC 0.25
#define KDEC 0.5

ros::Time lastcall;

ros::Publisher pub_odom; //declaration publisher
ros::Publisher pub_moteur; //declaration du publisher
std_msgs::MultiArrayDimension msg_dim; //variable de dimension du tableau
std_msgs::Float32MultiArray msg; //varriable du tableau qui publie les vitesses des moteurs
std_msgs::MultiArrayDimension odom_dim; //variable de dimension du tableau
std_msgs::Float32MultiArray odom; //variable du tableau qui publie l'odometrie
std_msgs::MultiArrayDimension val_encodeur_dim;
std_msgs::Float32MultiArray val_encodeur; //valeurs des vitesse calculees grace au encodeur

void callback( const std_msgs::Float32MultiArray& val)
{
  //ROS_INFO("---------------------");
  //recuperation des donnees lues sur le subscriber
  float vx = val.data[0]; //vx
  float vy = val.data[1]; //vy
  float w = val.data[2]; //w
  

  //rampe d'acceleration et de deceleration
  if(((vx-val_encodeur.data[0])<0 && val_encodeur.data[0] >0) || ((vx-val_encodeur.data[0])>0 && val_encodeur.data[0] <0))
  {
    vx = val_encodeur.data[0] + (vx - val_encodeur.data[0])*(KDEC);
  }else
  {
    vx = val_encodeur.data[0] + (vx - val_encodeur.data[0])*(KACC);
  }
  
  if(((vy-val_encodeur.data[1])<0 && val_encodeur.data[1] >0) || ((vy-val_encodeur.data[1])>0 && val_encodeur.data[1] <0))
  {
    vy = val_encodeur.data[1] + (vy - val_encodeur.data[1])*(KDEC);
  }else
  {
     vy = val_encodeur.data[1] + (vy - val_encodeur.data[1])*(KACC);
  }
  
  if(((w-val_encodeur.data[2])<0 && val_encodeur.data[2] >0) || ((w-val_encodeur.data[2])>0 && val_encodeur.data[2] <0))
  {
    w = val_encodeur.data[2] + (w - val_encodeur.data[2])*(KDEC);
  }else
  {
    w = val_encodeur.data[2] + (w - val_encodeur.data[2])*(KACC);
  }
  
  //on cap les vitesses
  if(fabs(vx)>VMAX)
  {
  	if(vx>0){vx=VMAX;}
  	else{vx=-VMAX;}
  }
  if(fabs(vy)>VMAX)
  {
  	if(vy>0){vy=VMAX;}
  	else{vy=-VMAX;}
  }
 if(fabs(vx)<VMIN)vx=0.0;
 if(fabs(vy)<VMIN)vy=0.0;
 if(fabs(w) < WMIN)w=0.0;
 if(fabs(w)>WMAX)
  {
  	if(w>0){w=WMAX;}
  	else{w=-WMAX;}
  }
    
 
  float vit4 = ((1/RAYON) * (vx + vy - w*(LONGUEUR + LARGEUR))); //correspond au moteur  w1 du modele
  float vit1 = ((1/RAYON) * (vx - vy + w*(LONGUEUR + LARGEUR))); //correspond au moteur  w2 du modele
  float vit3 = ((1/RAYON) * (vx - vy - w*(LONGUEUR + LARGEUR))); //correspond au moteur  w3 du modele
  float vit2 = ((1/RAYON) * (vx + vy + w*(LONGUEUR + LARGEUR))); //correspond au moteur  w4 du modele
 
  if(vit1 > VMAXMOT)
    vit1 = VMAXMOT;
  if(vit1 < -VMAXMOT)
    vit1 = -VMAXMOT;
  
  if(vit2 > VMAXMOT)
    vit2 = VMAXMOT;
  if(vit2 < -VMAXMOT)
    vit2 = -VMAXMOT;
  
  if(vit3 > VMAXMOT)
    vit3 = VMAXMOT;
  if(vit3 < -VMAXMOT)
    vit3 = -VMAXMOT;
  
  if(vit4 > VMAXMOT)
    vit4 = VMAXMOT;
  if(vit4 < -VMAXMOT)
    vit4 = -VMAXMOT;
 /* ROS_INFO("vitX envoyee = %f vitX encodeur = %lf\n", vx, val_encodeur.data[0]);
  ROS_INFO("vitY envoyee = %f vitY encodeur = %f\n", vy, val_encodeur.data[1]);
  ROS_INFO("Omega envoyee = %f Omega encodeur = %f\n", w , val_encodeur.data[2]);
  ROS_INFO("V1 = %f V2 = %f V3 %f V4 = %f", vit1, vit2, vit3, vit4);*/
  
  //publication sur valeur_moteur
  msg.data.clear();
  msg.data.push_back(vit1);
  msg.data.push_back(vit2);
  msg.data.push_back(vit3);
  msg.data.push_back(vit4);
  pub_moteur.publish(msg);
  
  //publication sur valeur_odom
  odom.data.clear();
  odom.data.push_back(vx);
  odom.data.push_back(vy);
  odom.data.push_back(w);
  pub_odom.publish(odom);
//   time_t* buffer;
  lastcall = ros::Time::now();
  
}

void callback2(const std_msgs::Float32MultiArray::ConstPtr& enc)
{
  std_msgs::Float32MultiArray val_enc = *enc;
  val_encodeur.data = val_enc.data;
} 

int main( int argc, char** argv )
{
  ros::init(argc, argv, "vitesse_moteur");

  ros::NodeHandle n; //declaration de nodehandle  
  //initilisation des dimensions du tableau
  msg_dim.label = "vitesses";
  msg_dim.size = 4;
  
  odom_dim.label = "odometrie";
  odom_dim.size = 3;
  
  val_encodeur_dim.label = "vitesseEncodeur";
  val_encodeur_dim.size = 3;
  
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);
  msg.data.clear();
  msg.data.push_back(0);
  msg.data.push_back(0);
  msg.data.push_back(0);
  msg.data.push_back(0);
  
  odom.layout.dim.clear();
  odom.layout.dim.push_back(odom_dim);
  
  val_encodeur.layout.dim.clear();
  val_encodeur.layout.dim.push_back(val_encodeur_dim);
  
  lastcall = ros::Time::now();
  
  //initialisation du publisher vitesse moteur
  pub_moteur = n.advertise<std_msgs::Float32MultiArray>("valeur_moteur", 1);
  
  //initialisation du publisher d'odom
  pub_odom = n.advertise<std_msgs::Float32MultiArray>("valeur_odom",1);
    ROS_INFO("CALLBACK2");
  for(int i = 0 ; i<3;i++)
  {
    val_encodeur.data.push_back(0);
  }
  
  //declaration et initilaisation du subscriber
  ros::Subscriber sub_coManip =  n.subscribe("valeurs_triplet", 1, callback);
  ros::Subscriber sub_vitEnc = n.subscribe("vit_encodeurs",1,callback2);
  
  ros::Rate r(100); // 100 hz
  ros::Duration time_diff;
  while (ros::ok())
  {
    //... do some work, publish some messages, etc. ...
    ros::spinOnce();
    r.sleep();
    time_diff = ros::Time::now() - lastcall;
    //bool speed = (msg.data[0] == 0 && msg.data[1] == 0 && msg.data[2] == 0 && msg.data[3] == 0);
    /*if(time_diff.sec > 0.5){
      //publication sur valeur_moteur de 0 pour un arret d'urgence
      msg.data.clear();
      msg.data.push_back(0);
      msg.data.push_back(0);
      msg.data.push_back(0);
      msg.data.push_back(0);
      pub_moteur.publish(msg);
    }*/
  }
//ros::spin();
  return 0;
}
  
  
  
  
  
  
