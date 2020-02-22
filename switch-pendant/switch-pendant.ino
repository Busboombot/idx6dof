#include <ros.h>
#include <std_msgs/String.h>

#include <Arduino.h>
#include "idx_pendant.h"

ros::NodeHandle nh;

IDXPendant pendant  = IDXPendant();

std_msgs::String str_msg;
ros::Publisher pendant_pub("sw_pendant", &str_msg);

long nextTime;

void setup()
{
  //Serial.begin(9600);
  nh.initNode();
  nh.advertise(pendant_pub);
  nh.loginfo("Starting pendant arduino node");
  pendant.begin() ;
  nextTime = millis();  
}

int pub_delay = 500;

void loop()
{

  if(pendant.run_once()){
      str_msg.data = pendant.outstr();
      pendant_pub.publish( &str_msg );
  }

  if( millis() - nextTime > pub_delay){
    str_msg.data = pendant.outstr();
    pendant_pub.publish( &str_msg );
    nextTime = millis();
  }
      
  nh.spinOnce();
  
}
