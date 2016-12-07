/**
\file    move_joint_node.cpp
\brief  Joint mover using key strokes as command
 *
 *  This node reads input from the keyboard and moves a joint of the Baxter robot,
 *  incrementing/decrementing its position according to the key that was hit.
 *  The name of the joint which is controlled is set using a ROS parameter.
\author  Gâetan Garcia
\date    30/7/2014
*/

//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"

//ROS msgs
#include <std_msgs/Char.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>

#define PLUS  (int(43)) //The ASCII code for the increment key
#define MINUS (int(45)) //The ASCII code for the decrement key
#define DEFAULT_INCR (double(5*M_PI/180.0))  /* 5 degrees */

//Namespaces
using namespace std;

//Global variables
ros::Publisher pub_command ;//The publisher of the command to move the joint

sensor_msgs::JointState last_state ;//Variable to store the last known joint state of the robot
string joint_name ; //To store the name of the joint to move
int incr_key, decr_key ;//The acii key codes to increment and decrement
double joint_incr ;//The increment value

/**
  \fn void jointStateCallback(sensor_msgs::JointState msg_state)
  \brief Captures the current joint state of the robot and stores it in a global variable to be accesible by the rest of functions
  */
void jointStateCallback(sensor_msgs::JointState msg_state)
{
  last_state.name.clear() ;
  last_state.position.clear() ;
  last_state.header = msg_state.header ;
  // Copy msg_state into global variable last_state.
  // Here, velocity and effort information are not used and left empty.
  for( unsigned i = 0 ; i < msg_state.name.size() ; i++ ) 
  {
    //cout << "joint_idx" << i << "joint name:" << msg_state.name[i] << endl;
    last_state.name.push_back( msg_state.name[i] ) ;
    last_state.position.push_back( msg_state.position[i] ) ;
  }
}

/**
  \fn void keyHitCallback(std_msgs::Int16 msg_key_hit)
  \brief Captures the key hit as a ROS topic and sends the corresponding joint command to move the predefined joint in position mode
  */
void keyHitCallback(std_msgs::Int16 msg_key_hit)
{
  unsigned joint_num=0 ;
  baxter_core_msgs::JointCommand msg_command;
  bool found=false;

  //cout << "Received: " << msg_key_hit.data << endl ;

  //Search for the joint to move

  for( int i=0 ; i < last_state.name.size(); i++ )
  {
    if (last_state.name[i]==joint_name) 
    {
     joint_num=i;
     found=true;
    }
  }

  if (found==false) 
  {
    cout << "Joint:" << joint_name << " not found!!!" << endl;
    return;
  }

  cout << "Joint name: " << joint_name << endl ;
  cout << "Joint number: " << joint_num << endl ;

  if(msg_key_hit.data==incr_key)
  {
    cout << "Moving up!" << endl ;
    msg_command.mode = msg_command.POSITION_MODE; 
    msg_command.names.push_back(joint_name);
    msg_command.command.push_back( last_state.position[joint_num]+joint_incr );//Increment the joint value
    pub_command.publish(msg_command);//Publish the command
  }
  else if(msg_key_hit.data==decr_key)
  {
    cout << "Moving down!" << endl ;
    msg_command.mode = msg_command.POSITION_MODE ;
    msg_command.names.push_back( joint_name ) ;
    msg_command.command.push_back( last_state.position[joint_num]-joint_incr ) ;//Decrement the joint value
    pub_command.publish( msg_command ) ;//Publish the command
  }
}

int main (int argc, char** argv)
{
  //ROS Initialization
  ros::init(argc, argv, "move_joint_node");
  ROS_INFO("Node move_joint_node connected to roscore");
  ros::NodeHandle nh_("~");//ROS Handler - local namespace.

  // Getting the joint name from a parameter
  if( !nh_.getParam("joint_name",joint_name) )
  {
    ROS_INFO("Couldn't find parameter: joint_name\n");
  }
  else
  {
    ROS_INFO("Controlling joint: %s\n",joint_name.c_str()) ;
  }
  // The next two parameters have default values.

  // Beware: the retrieved variables must be declared as C++ types, not ROS types...
  nh_.param("incr_key", incr_key, PLUS);
  ROS_INFO("Increment key: %d\n",incr_key) ;

  nh_.param("decr_key", decr_key, MINUS);
  ROS_INFO("Decrement key: %d\n",decr_key) ;

  nh_.param("joint_incr", joint_incr, DEFAULT_INCR);
  ROS_INFO("Joint increment: %f\n",joint_incr) ;

  //Subscribing
  ROS_INFO("Subscribing to topics\n");
  ros::Subscriber baxter_joint_state = nh_.subscribe<sensor_msgs::JointState> ("/robot/joint_states"  , 1, jointStateCallback);
  ros::Subscriber key_hit = nh_.subscribe<std_msgs::Int16> ("/key_hit"  , 1, keyHitCallback);

  //Publishing  
  pub_command = nh_.advertise<baxter_core_msgs::JointCommand>("/joint_command", 1);

  ros::spin();
/*
  ros::Rate rate(100);
  //ROS_INFO("SPINNING @ 100Hz");
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  } */

  ROS_INFO("ROS-Node Terminated\n");
}
