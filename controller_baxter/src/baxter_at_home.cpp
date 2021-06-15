/** @ package controller_baxter
* 
*  \file baxter_at_home.cpp
*  \brief this file checks if the baxter reached the home position
*
*  \author Francesco Ganci, Zoe Betta, Lorenzo Causa, Federico Zecchi
*  \version 1.0
*  \date 12/06/2021
*  \details
* 
*  Subscribes to: <BR>
*	 baxter_joint_states
*
*  Publishes to: <BR>
*	 none
*
*  Services: <BR>
*    baxter_at_home_server
*
*  Description: <BR>
*  	This node implements the behaviour to check if one arm, received as
*   input, is in the configuration of the start state. It returns true if
*   that is the case, false otherwise.
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <human_baxter_collaboration/BaxterTrajectory.h>
#include <human_baxter_collaboration/UnityTf.h>
#include <controller_baxter/at_home.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/RobotState.h>
#include "iostream"
#include <tf/tf.h>

//NAMESPACES
using namespace std;
using namespace moveit;
using namespace planning_interface;
using namespace geometry_msgs;

//GLOBAL VARIABLES
sensor_msgs::JointState messaggio_joint,left_arm_msg_joint,right_arm_msg_joint;

//SUBSCRIBER CALLBACKS
void joints_callback(const sensor_msgs::JointState& msg );

//SERVICES
bool server_callback(controller_baxter::at_home::Request &req,controller_baxter::at_home::Response &res);

//FUNCTIONS
bool very_close(double A ,double B);

/**
 * \brief: Main function 
 * \param argc, argv
 * \retval: 0
 * 
 * It initializes the ros node and all the needed subscribers and publishers,
 * it also initializes the server
 */
int main(int argc, char** argv)
{
// Initialize the ROS Node
ros::init(argc, argv, "baxter_at_home");
ros::NodeHandle n;
// Initialize the subscriber from baxter_joint_states
ros::Subscriber joint_state_sub = n.subscribe("baxter_joint_states", 10,joints_callback);
// Initialize the server baxter_at_home_server
ros::ServiceServer baxter_at_home_server = n.advertiseService("baxter_at_home_server",server_callback);

while(ros::ok()){
 ros::spinOnce();
 }
}

/**
 * \brief: called when new data are available on the topic /baxter_joint_states
 * \param msg : a sensor message
 * \retval: None
 * 
 * This function deletes redundant information and saves them in variables
 * used later in the scripts
 */
void joints_callback(const sensor_msgs::JointState& msg)
{
 // deletes useless information and saves the joint configuration in two variables
 // one for the right arm and one for the left arm
 right_arm_msg_joint=msg;
 right_arm_msg_joint.name.erase(right_arm_msg_joint.name.begin());
 right_arm_msg_joint.name.erase(right_arm_msg_joint.name.begin()+7,right_arm_msg_joint.name.begin()+18);
 right_arm_msg_joint.position.erase(right_arm_msg_joint.position.begin());
 right_arm_msg_joint.position.erase(right_arm_msg_joint.position.begin()+7,right_arm_msg_joint.position.begin()+18);

 left_arm_msg_joint=msg;
 left_arm_msg_joint.name.erase(left_arm_msg_joint.name.begin());
 left_arm_msg_joint.name.erase(left_arm_msg_joint.name.begin(),left_arm_msg_joint.name.begin()+7);
 left_arm_msg_joint.name.erase(left_arm_msg_joint.name.begin()+7,left_arm_msg_joint.name.begin()+11);
 left_arm_msg_joint.position.erase(left_arm_msg_joint.position.begin());
 left_arm_msg_joint.position.erase(left_arm_msg_joint.position.begin(),left_arm_msg_joint.position.begin()+7);
 left_arm_msg_joint.position.erase(left_arm_msg_joint.position.begin()+7,left_arm_msg_joint.position.begin()+11);

}

/**
 * \brief: called when a server request arrives
 * \param req : the command request
 * \param res : the command response
 * \retval: true
 * 
 * This function is the server callback, it checks the joints of the requested
 * arm and sees if the values are close enough to the start configuration.
 */
bool server_callback(controller_baxter::at_home::Request &req,controller_baxter::at_home::Response &res){
 // I initialize the boolen variable to false	
 res.at_home=false;	
 sensor_msgs::JointState messaggio_joint;
 // I check which arm I want to check if at home
 if(req.arm=="right") messaggio_joint=right_arm_msg_joint;
 if(req.arm=="left")  messaggio_joint=left_arm_msg_joint;
 if(req.arm!="left"&& req.arm!="right") return false;
 
 /*
 cout<<"joint0: "<<messaggio_joint.position[0]<<endl;
 cout<<"joint1: "<<messaggio_joint.position[1]<<endl;
 cout<<"joint2: "<<messaggio_joint.position[2]<<endl;
 cout<<"joint3: "<<messaggio_joint.position[3]<<endl;
 cout<<"joint4: "<<messaggio_joint.position[4]<<endl;
 cout<<"joint5: "<<messaggio_joint.position[5]<<endl;
 cout<<"joint6: "<<messaggio_joint.position[6]<<endl;
 */
 // I check if the distance is less than a given threshold for each joint
 // if that is the case I set res.at_home as true
 if((very_close(messaggio_joint.position[0],0.52)||very_close(messaggio_joint.position[0],-0.52))&& 
 very_close(messaggio_joint.position[1],-1.22)&& 
 very_close(messaggio_joint.position[2],0) && 
 very_close(messaggio_joint.position[3],1.72) && 
 very_close(messaggio_joint.position[4],0) && 
 very_close(messaggio_joint.position[5],0.75) && 
 very_close(messaggio_joint.position[6],0))
	{res.at_home=true;}
	
 return true;	
}

/**
 * \brief: checks if two joint values are close
 * \param A: joint value
 * \param B: joint value
 * \retval: true
 * 
 * We check if the difference between two numbers is between plus or minus 0.01
 */
bool very_close(double A ,double B){
// checks if the distance is less than 0.01 in absolute value	
if(A<B+0.01 && A>B-0.01)return true;
return false;		
}

