/** @ package controller_baxter
* 
*  \file go_to_home.cpp
*  \brief this file sets the home position if needed in the beginning
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
*	 baxter_moveit_trajectory
*
*  Services: <BR>
*    None
*
*  Description: <BR>
*  	This node implements 
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <human_baxter_collaboration/BaxterTrajectory.h>
#include <human_baxter_collaboration/UnityTf.h>
#include <controller_baxter/command.h>
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
MoveGroupInterface *right_arm;
MoveGroupInterface *left_arm;
MoveGroupInterface *both_arms;
ros::Publisher trajectory_pub;
sensor_msgs::JointState left_arm_msg_joint,right_arm_msg_joint;

//SUBSCRIBER CALLBACKS
void joints_callback(const sensor_msgs::JointState& msg );

//FUNCTIONS DECLARATION
void move_to_home();
void update_start_state();
void initialize_joints();

/**
 * \brief: Main function 
 * \param argc, argv
 * \retval: 0
 * 
 * It initializes the ros node and all the needed subscribers and publishers.
 */
int main(int argc, char** argv)
{

// Initialize the ROS Node
ros::init(argc, argv, "go_to_home");
ros::NodeHandle n;
// Initialize the subscriber from baxter_joint_states
ros::Subscriber joint_state_sub = n.subscribe("baxter_joint_states", 10,joints_callback);
// Initialize the publisher for the moveit trajectory
trajectory_pub = n.advertise<human_baxter_collaboration::BaxterTrajectory>("baxter_moveit_trajectory", 10);
// Initialize the spinner
static ros::AsyncSpinner spinner(4);
spinner.start();
ros::WallDuration(1.0).sleep();

// Initialization of all the moveit groups
MoveGroupInterface movegroup_right("right_arm");
MoveGroupInterface movegroup_left("left_arm");

right_arm=&movegroup_right;
left_arm=&movegroup_left;

// Initialization of the joint position vectors
vector<double> joints_right;
vector<double> joints_left;

// Getting the current position for all the joints
joints_right=movegroup_right.getCurrentJointValues();
joints_left=movegroup_left.getCurrentJointValues();

sleep(3);
update_start_state();
move_to_home();

spinner.stop();
return 0;
}

/**
 * \brief:moves the baxter to the home position
 * \param None
 * \retval: None
 * 
 * This function calculates the home position by manually setting the values,
 * it then calculates the moveit trajectory and publishes it
 */
void move_to_home()
{
  MoveGroupInterface::Plan my_plan;
  bool success;
  vector<double> joint_position_right,joint_position_left;
  // set the joint positions
  joint_position_left.push_back(-0.52);
  joint_position_left.push_back(-1.22);
  joint_position_left.push_back(0);
  joint_position_left.push_back(1.72);
  joint_position_left.push_back(0);
  joint_position_left.push_back(0.75);
  joint_position_left.push_back(0);

  joint_position_right.push_back(0.52);
  joint_position_right.push_back(-1.22);
  joint_position_right.push_back(0);
  joint_position_right.push_back(1.72);
  joint_position_right.push_back(0);
  joint_position_right.push_back(0.75);
  joint_position_right.push_back(0);

  // I calculate the trajectory for the right arm
  right_arm->setJointValueTarget(joint_position_right);
  success = (right_arm->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Plan Result:%s", success ? "SUCCESS" : "FAILED");
  // If I don't have a feasible trajectory 
  if(!success)printf("ERROR");

  // I save the trajectory in a queue
  human_baxter_collaboration::BaxterTrajectory my_trajectory_right;
  my_trajectory_right.arm="right";
  my_trajectory_right.trajectory.push_back(my_plan.trajectory_);
  // I publish the trajectory
  trajectory_pub.publish(my_trajectory_right);

  // I calculate the trajectory for the left arm
  left_arm->setJointValueTarget(joint_position_left);
  success = (left_arm->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Plan Result:%s", success ? "SUCCESS" : "FAILED");
  // If I don't have a feasible trajectory 
  if(!success)printf("ERROR");

  // I save the tracjectory in a queue
  human_baxter_collaboration::BaxterTrajectory my_trajectory_left;
  my_trajectory_left.arm="left";
  my_trajectory_left.trajectory.push_back(my_plan.trajectory_);
  // I publish the trajectory
  trajectory_pub.publish(my_trajectory_left);

}

/**
 * \brief : updates the start state
 * \param none
 * \retval : none
 * 
 * This function retrieves the position of the joints of the robot in the 
 * beginning and sets them as the initial configuration to start the trajectory from.
 */
void update_start_state(){
  //Updating the right arm: it retrieves the current position and sets it as the start state	
  robot_state::RobotState start_state_right(*right_arm->getCurrentState());
  const robot_state::JointModelGroup *joint_model_group_right =start_state_right.getJointModelGroup(right_arm->getName());
  start_state_right.setJointGroupPositions(joint_model_group_right, right_arm_msg_joint.position);
  right_arm->setStartState(start_state_right);

  //Updating the left arm: it retrieves the current position and sets it as the start state
  robot_state::RobotState start_state_left(*left_arm->getCurrentState());
  const robot_state::JointModelGroup *joint_model_group_left =start_state_left.getJointModelGroup(left_arm->getName());
  start_state_left.setJointGroupPositions(joint_model_group_left, left_arm_msg_joint.position);
  left_arm->setStartState(start_state_left);
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

