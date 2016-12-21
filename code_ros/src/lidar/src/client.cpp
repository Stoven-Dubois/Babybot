 /*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved. 
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  RPlidar ROS Node client test app
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */

//Programme qui supprime les éléments du robot du signal du lidar

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32MultiArray.h"
#define RAD2DEG(x) ((x)*180./M_PI)

using namespace std;
ros::Publisher lidar_pub;

// conversion coordonnées polaire -> cartesienne
void convert(double r, float t, int *x ,int *y)
{
    double pi = 3.141592654;
    *x = (int)(r)*cos((t*pi/180 + pi/2));
    *y = (int)(r)*sin((t*pi/180+pi/2));
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    sensor_msgs::LaserScan scanTraite = *scan;
    std_msgs::Int32MultiArray msg;
    int count = scan->scan_time / scan->time_increment;
    int x,y; // valeur coordonée cartésienne
    int coef = 100; // coefficient pour conversion x1(m), x10(dm), x100(cm), x1000(mm)
    float D; // distance en m*coef
    int i;
    int av = 0,ar = 0, ga = 0, dr =0;
    float angle_mort = 4/2;
    float D_min1 =25.5, D_min2 = 40.5, D_max = 700;// en cm
    int marge = 10; // marge de detection
    float D_detection1 = D_min1 + marge, D_detection2 = D_min2 + marge ;
    int max[4] = {0,0,0,0},dir =0;
    float largeur =50, longueur = 80, distanceLidarAvant=30;
    float margeCote = 2;
    // on regarde pour chaque information donnée par le LiDAR
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);// degree lu
        D = (scan->ranges[i]*coef); // distance lue
	convert(D,degree,&x,&y); // conversion en cartesien
	// si la distance lue est inferieur à la distance max
	if( D < D_max){
		//Si le signal est dans l'espace du robot
		if(y>(-(largeur/2 + margeCote)) && y<((largeur/2)/2) && x>(-(distanceLidarAvant+margeCote)) && x<(longueur - distanceLidarAvant + margeCote)){
			scanTraite.ranges[i] = 0;
		}
	}
    }
    // on met à 0 les max s'il y a un obstacles
    max[0] = -1*max[0]*(ga-1);
    max[1]= -1*max[1]*(av-1);
    max[2] = -1*max[2]*(dr-1);
    max[3] = -1*max[3]*(0); // on ne regarde pas en arrière
    // on trouve le max dans le tableau 
    int MIN=0;
    int MAX=0;
    for (i=0; i<4; i++)
     {
      if(max[i]>max[MAX]) MAX=i;
      if(max[i]<max[MIN]) MIN=i;
     }
    // envoie des données sur le topic lidar_talker
    msg.data.clear();
    msg.data.push_back(av);
    msg.data.push_back(ar);
    msg.data.push_back(ga);
    msg.data.push_back(dr);
    msg.data.push_back(MAX+1);
    lidar_pub.publish(scanTraite);    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_client");
    ros::NodeHandle n;
    lidar_pub = n.advertise<sensor_msgs::LaserScan>("/lidar_talker",5);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, scanCallback);

    ros::spin();

    return 0;
}
