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

int main(int argc, char** argv)
{

// Initialize the ROS Node
ros::init(argc, argv, "go_to_home");
ros::NodeHandle n;

ros::Subscriber joint_state_sub = n.subscribe("baxter_joint_states", 10,joints_callback);

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

/***
 * @brief : This function moves the requested block inside the blue box
 * @param block_name : the name of the block I want to move
 * @param group : with which arm I want to move
 * @retval : it returns true if we were able to complete the movement or false if not
 ***/
void move_to_home()
{
  MoveGroupInterface::Plan my_plan;
  bool success;
  vector<double> joint_position_right,joint_position_left;

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

  right_arm->setJointValueTarget(joint_position_right);
  success = (right_arm->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Plan Result:%s", success ? "SUCCESS" : "FAILED");
  // If I don't have a feasible trajectory I return false
  if(!success)printf("ERROREEEEEEEEEE");

  // I save the tracjectory in a queue
  human_baxter_collaboration::BaxterTrajectory my_trajectory_right;
  my_trajectory_right.arm="right";
  my_trajectory_right.trajectory.push_back(my_plan.trajectory_);
  trajectory_pub.publish(my_trajectory_right);


  left_arm->setJointValueTarget(joint_position_left);
  success = (left_arm->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Plan Result:%s", success ? "SUCCESS" : "FAILED");
  // If I don't have a feasible trajectory I return false
  if(!success)printf("ERROREEEEEEEEEE");

  // I save the tracjectory in a queue
  human_baxter_collaboration::BaxterTrajectory my_trajectory_left;
  my_trajectory_left.arm="left";
  my_trajectory_left.trajectory.push_back(my_plan.trajectory_);
  trajectory_pub.publish(my_trajectory_left);

}

void update_start_state(){
  robot_state::RobotState start_state_right(*right_arm->getCurrentState());
  const robot_state::JointModelGroup *joint_model_group_right =start_state_right.getJointModelGroup(right_arm->getName());
  start_state_right.setJointGroupPositions(joint_model_group_right, right_arm_msg_joint.position);
  right_arm->setStartState(start_state_right);


  robot_state::RobotState start_state_left(*left_arm->getCurrentState());
  const robot_state::JointModelGroup *joint_model_group_left =start_state_left.getJointModelGroup(left_arm->getName());
  start_state_left.setJointGroupPositions(joint_model_group_left, left_arm_msg_joint.position);
  left_arm->setStartState(start_state_left);
}

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

