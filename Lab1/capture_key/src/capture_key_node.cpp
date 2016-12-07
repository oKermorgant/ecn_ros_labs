/**
\file    capture_key_node.cpp
\brief   Publishes ROS topics containing the code of key strokes
\author  Gâetan Garcia
\date    30/7/2014
*/
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>

#include <algorithm>
#include <stdlib.h>

//ROS
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std ;

/** \brief Captures any keystroke from the keyboard
  *
  * \returns int : the ASCII code of the key pressed
  */
int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	return ch;
}


int main (int argc, char** argv)
{
  int key;
  ros::Publisher pub_key;

  //Connect to ROS
  ros::init(argc, argv, "capture_key_node");
  ROS_INFO("Node capture_key_node connected to roscore");

  ros::NodeHandle nh_;//ROS Handler

  //Registers the topic that will contain the keystroke
  pub_key = nh_.advertise<std_msgs::Int16>("/key_typed", 1);

  ros::Rate rate(100);//Defines the rate of the loop
  while (ros::ok())
  {
   std_msgs::Int16 key_typed ;//Topic variable to store the result

   ros::spinOnce();//Listen for any incomming topic, just put it as a good practice even if not subscribed to any topic
   key=kbhit();//Catch the key if any

   if( key > 0 )//Only publish valid key strokes
   {
      key_typed.data=key ;//Assign value
      pub_key.publish(key_typed);//Publish it
   }

   rate.sleep();//Wait for loop time to complete
  }
  ROS_INFO("ROS-Node Terminated\n");
}
