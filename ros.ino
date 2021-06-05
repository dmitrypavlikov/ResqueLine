/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

ros::NodeHandle  nh;
void messageCb( const std_msgs::Int32& toggle_msg){
  digitalWrite(2, HIGH);   // blink the led
  delay(200);
  digitalWrite(2, LOW);
}
ros::Subscriber<std_msgs::Int32> sub("raspberry", &messageCb );

std_msgs::Int32 int_msg; 
ros::Publisher chatter("opercr", &int_msg);

char hello[13] = "hello world!";



void setup()
{
  pinMode(7,INPUT_PULLUP);
  pinMode(2,OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}
void send_msg_to_topic(){
  if(digitalRead(7) == LOW){
  int_msg.data = 777;
  chatter.publish(&int_msg);
  }
  nh.spinOnce();
    
  }
void loop()
{

 send_msg_to_topic();
  delay(100);
}

