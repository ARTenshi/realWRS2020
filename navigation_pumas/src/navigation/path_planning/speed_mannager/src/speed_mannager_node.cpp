#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "tf/transform_listener.h"
#include "std_msgs/Float32.h"

geometry_msgs::Twist speed_from_path;
geometry_msgs::Twist speed_from_hand;
geometry_msgs::Twist speed_result;
float   alpha             = 0.4;                 //Its the force of the robots path [1.0 means totally robot control, 0.0 totally driven by human]
float   alpha_human_speed = 0.0;
float   scale_hand        = 2.0;
bool    msg_recived;


////////////////////////////////////////////////////////////////
///          Callbacks for Subscribers
////////////////////////////////////////////////////////////////

void callback_speed_path(const geometry_msgs::Twist::ConstPtr& msg)
{
    std::cout << "msg msg_recived... " << std::endl;
    speed_from_path.linear   = msg->linear;
    speed_from_path.angular  = msg->angular;


    std::cout << "msg:  " << speed_from_path.linear << std::endl;
    msg_recived = true;
}

void callback_force_hand(const geometry_msgs::Twist::ConstPtr& msg)
{
     speed_from_hand.linear  = msg->linear;
     speed_from_hand.angular = msg->angular;
     msg_recived = true;
}

void callback_human_speed(const geometry_msgs::Twist::ConstPtr& msg)
{
  alpha_human_speed = msg->linear.x;
  msg_recived = true;
}


geometry_msgs::Twist update_speeds_path_hand(geometry_msgs::Twist speed_from_path_temp, geometry_msgs::Twist speed_from_hand_temp)
{
  speed_result.linear.x = (speed_from_path_temp.linear.x * alpha) + ((1-alpha) * speed_from_hand_temp.linear.x * scale_hand);
  speed_result.linear.y = (speed_from_path_temp.linear.y * alpha) + ((1-alpha) * speed_from_hand_temp.linear.y * scale_hand );
  speed_result.angular.z = (speed_from_path_temp.angular.z * alpha) + ((1-alpha) * speed_from_hand_temp.angular.z * scale_hand);
  
  // std::cout << "msg_result:  " << speed_result.linear << std::endl;
  return speed_result;
}

geometry_msgs::Twist update_speeds_path_human(geometry_msgs::Twist speed_from_path_temp)
{
  speed_result.linear.x  = speed_from_path_temp.linear.x * alpha_human_speed * 3;
  speed_result.linear.y  = speed_from_path_temp.linear.y;
  speed_result.angular.z = speed_from_path_temp.angular.z;
  
  // std::cout << "msg_result:  " << speed_result.linear << std::endl;
  return speed_result;
}




int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A SPEED MANNAGER FOR ROBOT COOPERATIVE TASK by EDD-2..." << std::endl;

    //
    //VARIABLES FOR ROS CONNECTION
    ros::init(argc, argv, "speed_mannager");
    ros::NodeHandle n;                           
    ros::Publisher  pub_cmd_vel           = n.advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel", 1);
    ros::Subscriber sub_speed_path        = n.subscribe("/hardware/mobile_base/cmd_vel_path", 1, callback_speed_path);
    ros::Subscriber sub_force_hand        = n.subscribe("/hardware/hand_force", 1, callback_force_hand);
    ros::Subscriber sub_human_speed       = n.subscribe("/hri/human_speed_calculator/human_speed", 1, callback_human_speed);
    ros::Rate loop(20);
	    
    while(ros::ok())
    {
      if(msg_recived)
      {
	// pub_cmd_vel.publish( update_speeds_path_hand(speed_from_path, speed_from_hand) );
	pub_cmd_vel.publish( update_speeds_path_human(speed_from_path) );
	msg_recived = false;
      }
      
      ros::spinOnce();
      loop.sleep();
    }
    return 0;
}
