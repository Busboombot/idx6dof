#include <ros.h>
#include <std_msgs/String.h>

#include <Arduino.h>
#include "idx_pendant.h"

ros::NodeHandle nh;

IDXPendant pendant  = IDXPendant();

std_msgs::String str_msg;
ros::Publisher pendant_pub("sw_pendant", &str_msg);

long nextTime;

#define USE_SERIAL true

//
// ROS Version
// 
void setup()
{
#ifdef USE_SERIAL
  Serial.begin(9600);
  Serial.println("Starting pendant");
#else
  nh.initNode();
  nh.advertise(pendant_pub);
  nh.loginfo("Starting pendant arduino node");
#endif
  pendant.begin() ;
  nextTime = millis(); 
   
}

int pub_delay = 500;

void loop()
{

  if(pendant.run_once()){
      str_msg.data = pendant.outstr();
      nextTime-=pub_delay;
  }

  if( millis() - nextTime > pub_delay){
#ifdef USE_SERIAL
    Serial.println(str_msg.data);
#else
    pendant_pub.publish( &str_msg );
#endif
    nextTime = millis();
  }

#ifndef USE_SERIAL
  nh.spinOnce();
#endif
  
}
