#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/transform_listener.h>
#include <ctime>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#define PI 3.14159265358979323846

tf::StampedTransform transform_robot_code, transform_robot_camera, transform_code_rotation, transform_code_statique, transform_tag_in_map; //Transformations entre les frames
ros::Publisher camera_code_pub;
ros::Subscriber sub;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
time_t timerPrecedent, timerActuel; //Variable de temps pour envoyer un objectif 1 fois par seconde

float x_envoi = 0, y_envoi = 0; //Coordonnées de l'objectif dans /map
float x = 0, y = 0, angleCode = 0; //Données envoyées sur camera_code_pub pour l'asservissement de la caméra
int test; //Variable qui vaut 1 si la caméra perçoit le code, 0 sinon
ar_track_alvar_msgs::AlvarMarkers frame;

std_msgs::Int32MultiArray code_pixel;

void callback(ar_track_alvar_msgs::AlvarMarkers::ConstPtr markers){
  test = 0;
  for(int i=0; i<markers->markers.size();i++){ //On regarde si la caméra repère le code
    if(markers->markers[i].id == 200){
      test = 1;
      break;
    }
  }   
}

int main(int argc, char** argv){
  ros::init(argc, argv, "camera_envoi_objectif");
  ros::NodeHandle node;
  camera_code_pub = node.advertise<std_msgs::Int32MultiArray>("/Servo", 3);
  ros::Subscriber sub = node.subscribe("/ar_pose_marker", 1, callback);
  
  //On initialise les variables de temps
  time(&timerActuel);
  time(&timerPrecedent);

  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::TransformListener li;
  
  //Déclarations pour l'objectif
  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  
  ros::Rate rate(10.0);
  
  while(ros::ok()){
    /*ROS_ERROR("***************");
    //Récupération des transformations entre les frames
    */try{
      li.lookupTransform("/head_camera", "/ar_marker_200", ros::Time(0), transform_code_rotation);
    }
    catch (tf::TransformException ex){
	//ROS_ERROR("%s",ex.what());
	continue;
    }
    
    try{
      li.lookupTransform("/camera_base", "/ar_marker_200", ros::Time(0), transform_code_statique);
    }
    catch (tf::TransformException ex){
	//ROS_ERROR("%s",ex.what());
	continue;
    }
    
    try{
      li.lookupTransform( "/map", "/head_camera", ros::Time(0), transform_robot_camera);
    }
    catch (tf::TransformException ex){
	//ROS_ERROR("%s",ex.what());w
	continue;
    }
    
    try{
      li.lookupTransform( "/map", "/ar_marker_200", ros::Time(0), transform_tag_in_map);

    }
    catch (tf::TransformException ex){
	//ROS_ERROR("a");
	continue;
    }
    //ROS_ERROR("b\n");
      //Calcul des coordonnées x et y de l'objectif dans le repère "/camera_base" et de son angle pour l'asservissement de la caméra
      x = transform_code_statique.getOrigin().getX();
      y = transform_code_statique.getOrigin().getY();
      angleCode = (int) (atan2(y, x)*180/PI + 90);
      
      //Calcul des coordonnées de l'objectif dans "/map"
      transform_robot_camera *= transform_code_rotation;
      x_envoi = transform_robot_camera.getOrigin().getX();
      y_envoi = transform_robot_camera.getOrigin().getY();
      ROS_INFO("x : %f, y : %f, angle : %f\n", x_envoi, y_envoi, angleCode);
      
      //Envoi de l'objectif
//       goal.target_pose.pose.position.x = x_envoi;
//       goal.target_pose.pose.position.y = y_envoi;
      goal.target_pose.pose.position.x = transform_tag_in_map.getOrigin().getX();
      goal.target_pose.pose.position.y = transform_tag_in_map.getOrigin().getY();
      goal.target_pose.pose.position.z = 1.19;
   
      //On envoi une rotation quelconque puisque le robot a 2PI de tolérance pour son orientation finale
      goal.target_pose.pose.orientation.x = 0;
      goal.target_pose.pose.orientation.y = 0;
      goal.target_pose.pose.orientation.z = 0;
      goal.target_pose.pose.orientation.w = 1;
      
      //Remise à zero
      x = 0;
      y = 0;
      x_envoi = 0;
      y_envoi = 0;
      
      
      code_pixel.data.clear();
      time(&timerActuel);
      if(test != 0){ //Si la caméra repère le code
	code_pixel.data.push_back(1);
        code_pixel.data.push_back(0);
	code_pixel.data.push_back(angleCode);
	if(difftime(timerActuel, timerPrecedent) > 1){ //Si ça fait plus d'une seconde depuis le dernier envoi d'objectif
	  ac.sendGoal(goal);
	  ROS_INFO("Sending goal");  
	  time(&timerPrecedent);
	}
      }else{ //Si la caméra ne repère pas le code
	code_pixel.data.push_back(0);
	code_pixel.data.push_back(0);
	code_pixel.data.push_back(0);
      }
    camera_code_pub.publish(code_pixel);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}