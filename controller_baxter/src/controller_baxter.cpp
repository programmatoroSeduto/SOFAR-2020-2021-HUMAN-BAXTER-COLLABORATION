/** @ package controller_baxter
* 
*  \file controller_baxter.cpp
*  \brief this file implements the generation of the trajectory
*
*  \author Francesco Ganci, Zoe Betta, Lorenzo Causa, Federico Zecchi
*  \version 1.0
*  \date 12/06/2021
*  \details
* 
*  Subscribes to: <BR>
*	 unity_tf
*
*  Publishes to: <BR>
*	 baxter_moveit_trajectory
*
*  Services: <BR>
*    controller_server
*
*  Description: <BR>
*  	This node implements a server that receives as input which arm needs to move,
*   which block and where it needs to put the block. It sets all the information and then it 
*   calculates the trajectory with moveit. The trajectory is divided in 
*   5 phases: it first moves the gripper above the block, it moves down to pick the 
*   block, it moves above the block again, it moves on top of the goal and it
*   goes down to the goal. 
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

//DEFINE VARIABLES
#define step_near_block 0.05
#define step_above_block 0.2 
#define step_above_final_pos 0.15
#define step_near_final_pos 0.05
#define step_optimize_grasp 0.01

//GLOBAL VARIABLES
MoveGroupInterface *right_arm;
MoveGroupInterface *left_arm;
MoveGroupInterface *both_arms;
ros::Publisher trajectory_pub;
sensor_msgs::JointState messaggio_joint,left_arm_msg_joint,right_arm_msg_joint,both_arm_msg_joint;
Pose target_block,red_box_pose,blue_box_pose,A_red,B_red,C_blue,D_red,E_blue,F_red,G_blue,H_red,I_blue,L_red,M_blue,target_for_arm_switch;
Pose C_blue_final,E_blue_final,G_blue_final,I_blue_final,M_blue_final;

//SUBSCRIBER CALLBACKS
void tf_callback(const human_baxter_collaboration::UnityTf& msg);

//SERVICES
bool server_callback(controller_baxter::command::Request &req,controller_baxter::command::Response &res);

//FUNCTIONS DECLARATION	
void print_all_joints();
double rad_to_grad(double rad);
void move_to_pose(geometry_msgs::Pose pt, bool Orientamento);
void stampa_Pose(Pose po);
void move_to_waypoints(vector<Pose> waypoints);
bool move_block(string block_name,MoveGroupInterface *group);
void update_start_state_at_home();
void update_start_state_after_trajectory_execution(moveit_msgs::RobotTrajectory last_trajectory,MoveGroupInterface *group);
Pose pose_of_block(string block_name);
bool move_block_to_switchpoint(string block_name,MoveGroupInterface *group);
Pose final_pose_of_block(string block_name);
void initialize_joints();

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
ros::init(argc, argv, "baxter_controller");
ros::NodeHandle n;
// Initialize the subscriber from unity tf
ros::Subscriber tf_sub = n.subscribe("unity_tf", 10, tf_callback);
// Initialize the server for controlling the baxter
ros::ServiceServer controller_server = n.advertiseService("controller_server",server_callback);
// Initialize the publisher for the moveit trajectory
trajectory_pub = n.advertise<human_baxter_collaboration::BaxterTrajectory>("baxter_moveit_trajectory", 10);
// Initialize the spinner
static ros::AsyncSpinner spinner(4);
spinner.start();
ros::WallDuration(1.0).sleep();

// Initialization of all the moveit groups
MoveGroupInterface movegroup_right("right_arm");
MoveGroupInterface movegroup_left("left_arm");
MoveGroupInterface movegroup_both("both_arms");
right_arm=&movegroup_right;
left_arm=&movegroup_left;
both_arms=&movegroup_both;
// Initialization of the joint position vectors
vector<double> joints_right;
vector<double> joints_left;
vector<double> joints_both;
// Getting the current position for all the joints
joints_right=movegroup_right.getCurrentJointValues();
joints_left=movegroup_left.getCurrentJointValues(); 
joints_both=movegroup_both.getCurrentJointValues();
initialize_joints();
update_start_state_at_home();

while(ros::ok()){
ros::spinOnce();
}

spinner.stop();
return 0;
}

/**
 * \brief: moves the block
 * \param block_name : the name of the block I want to move
 * \param str_final_pos : where I want to put the block
 * \retval: it returns true if we were able to complete the movement or false if not
 * 
 * it moves the block to a given target by dividing the trajectory in 
 * 5 phases. At the end, after setting the trajectory, the start position 
 * of the robot is set as the start position.
 */
bool move_block(string block_name,MoveGroupInterface *group,string str_final_pos)
{
  Pose final_target;
  //retrieving the coordinates for the block I want to move
  target_block=pose_of_block(block_name);

  // Identify which arm I want to use
  string arm;
  if(group->getName()=="right_arm"){
    arm="right";
  }

  if(group->getName()=="left_arm"){
    arm="left";
  }

  // Identify where I want to put the block
  if(str_final_pos=="box"){
    final_target=final_pose_of_block(block_name);
  }
  else if(str_final_pos=="center"){
    final_target=target_for_arm_switch;
  }
  else {
    ROS_INFO("NO FINAL POSITION KNOWN");
    return false;
  }

  ROS_INFO("Trying to execute a trajectory");
  Pose above_target;
  MoveGroupInterface::Plan my_plan;
  bool success;

  // setting the first position as the position right above the block
  // the x and y coordinates are the same of the block while the z coordinate is incremented
  above_target=target_block;
  above_target.position.z+=step_near_block;

  // I set the target pose as the one calculated before and I find the trajectory
  group->setPoseTarget(above_target);
  success = (group->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Plan Result:%s", success ? "SUCCESS" : "FAILED");
  // If I don't have a feasible trajectory I return false
  if(!success)return false;
  
  // I save the tracjectory in a queue
  human_baxter_collaboration::BaxterTrajectory my_trajectory;
  my_trajectory.arm=arm;
  my_trajectory.trajectory.push_back(my_plan.trajectory_);
  
  // I update the start state with the one just reached
  update_start_state_after_trajectory_execution(my_plan.trajectory_,group);

  // setting the next step to the position of the block
  Pose adjusted_target_block=target_block;
  adjusted_target_block.position.z-=step_optimize_grasp;

  // I set the target pose as the one of the block and I find the trajectory
  group->setPoseTarget(adjusted_target_block);
  success = (group->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Plan Result:%s", success ? "SUCCESS" : "FAILED");
  // If I don't have a feasible trajectory I return false
  if(!success)return false;
  
  // I save the tracjectory in a queue
  my_trajectory.trajectory.push_back(my_plan.trajectory_);
  
  // I update the start state with the one just reached
  update_start_state_after_trajectory_execution(my_plan.trajectory_,group);

  // set the next step as the position above the block
  Pose target_above_block=target_block;
  target_above_block.position.z+=step_above_block;

  // I set the target pose and I find the trajectory
  group->setPoseTarget(target_above_block);
  success = (group->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Plan Result:%s", success ? "SUCCESS" : "FAILED");
  // If I don't have a feasible trajectory I return false
  if(!success)return false;
  
  // I save the tracjectory in a queue
  my_trajectory.trajectory.push_back(my_plan.trajectory_);

  // I update the start state with the one just reached
  update_start_state_after_trajectory_execution(my_plan.trajectory_,group);

  // set the next step as above the final target
  Pose before_final_target=final_target;
  before_final_target.position.z+=step_above_final_pos;

  // I set the target pose and find the trajectory
  group->setPoseTarget(before_final_target);
  success = (group->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Plan Result:%s", success ? "SUCCESS" : "FAILED");
  // If I don't have a feasible trajectory I return false
  if(!success)return false;
  
  // I save the tracjectory in a queue
  my_trajectory.trajectory.push_back(my_plan.trajectory_);

  // I update the start state with the one just reached
  update_start_state_after_trajectory_execution(my_plan.trajectory_,group);

  // I set the next step as near the final target
  above_target=final_target;
  above_target.position.z+=step_near_final_pos;

  // I set the target pose and I find the trajectory 
  group->setPoseTarget(above_target);
  success = (group->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Plan Result:%s", success ? "SUCCESS" : "FAILED");
  // If I don't have a feasible trajectory I return false
  if(!success)return false;
  
  // I save the tracjectory in a queue
  my_trajectory.trajectory.push_back(my_plan.trajectory_);
  
  // I publish the trajectory to be executed
  trajectory_pub.publish(my_trajectory);
  
  // I update the start state with the home
  update_start_state_at_home();
  return true;
}

/**
 * \brief: finds the position of a block
 * \param block_name : the block I want to know the pose of
 * \retval: the Pose 
 * 
 * Depending on the name of the block given as input it returns the position
 */
Pose pose_of_block(string block_name){
  // I check which box I want the pose of
  if(block_name=="E"){

    return E_blue;

  }
  if(block_name=="M"){

    return M_blue;

  }
  if(block_name=="C"){

    return C_blue;

  }
  if(block_name=="G"){

    return G_blue;

  }
  if(block_name=="I"){

    return I_blue;

  }
  // If I can't find the block string I return the position of the M box
  else {
    ROS_INFO("ERROR, no block string found");
    return M_blue;
  }
}

/**
 * \brief: the final position of a block
 * \param block_name : the block I want to know the pose of
 * \retval: the Pose 
 * 
 * Depending on the name passed as input it returns where the final position should be.
 * the final position is different in each box in order to have them organized
 */
Pose final_pose_of_block(string block_name){
  // I check which box I want the final pose of
  if(block_name=="E"){

    return E_blue_final;

  }
  if(block_name=="M"){

    return M_blue_final;

  }
  if(block_name=="C"){

    return C_blue_final;

  }
  if(block_name=="G"){

    return G_blue_final;

  }
  if(block_name=="I"){

    return I_blue_final;

  }
  // If I can't find the block string I return the position of the M box
  else {
    ROS_INFO("ERROR, no block string found");
    return M_blue_final;
  }
}

/**
 * \brief: This function is called when you data are published on the topic unity_tf
 * \param msgs : the new data retrieved
 * \retval: none
 * 
 * After receiving the data on the tf callback I divide them and save them
 * in the correct global variable.
 */
void tf_callback(const human_baxter_collaboration::UnityTf& msg)
{
  // for the lenght og the message I save the frames in the corresponding box variable
  for(unsigned long i=0;i<msg.frames.size();i++){
    if(msg.frames[i].header.frame_id=="E"){

      E_blue=msg.frames[i].pose;

    }
    if(msg.frames[i].header.frame_id=="M"){

      M_blue=msg.frames[i].pose;

    }
    if(msg.frames[i].header.frame_id=="G"){

      G_blue=msg.frames[i].pose;

    }
    if(msg.frames[i].header.frame_id=="I"){

      I_blue=msg.frames[i].pose;

    }
    if(msg.frames[i].header.frame_id=="C"){

      C_blue=msg.frames[i].pose;


    }
    // for the frame of the red box I set its position 0.15 above the given one
    if(msg.frames[i].header.frame_id=="Redbox"){

      red_box_pose=msg.frames[i].pose;
      red_box_pose.position.z+=0.15;

    }
   // for the frame of the blue box I set its position 0.15 above the given one
    if(msg.frames[i].header.frame_id=="Bluebox"){

      blue_box_pose=msg.frames[i].pose;
      blue_box_pose.position.z+=step_near_final_pos;

      C_blue_final=blue_box_pose;
      E_blue_final=blue_box_pose;
      I_blue_final=blue_box_pose;
      G_blue_final=blue_box_pose;
      M_blue_final=blue_box_pose;

      //0.05 cubo

      C_blue_final.position.x+=0.07;
      C_blue_final.position.y-=0.03;


      I_blue_final.position.x+=0.07;
      I_blue_final.position.y+=0.03;

      G_blue_final.position.y-=0.03;

      E_blue_final.position.y+=0.03;

      M_blue_final.position.x-=0.07;

    }
    // for the frame of the center point I set its position 0.15 above the given one
    if(msg.frames[i].header.frame_id=="MiddlePlacementN"){

      target_for_arm_switch=msg.frames[i].pose;
      target_for_arm_switch.position.y+=0.1;

    }

  }

}


/**
 * \brief: This function is called to convert from radians to degrees
 * \param rad: the angle in radians
 * \retval: the angle in degrees
 */
double rad_to_grad(double rad)
{
return rad*180/3.1415;
}


/**
 * \brief:updates the start state of the robot
 * \param none
 * \retval: none
 * 
 * It sets the start state of the robot for the planning of the trajectory
 */
void update_start_state_at_home(){
  
  //Updating the right arm: it retrieves the current position and sets it as the start state
  robot_state::RobotState right_start_state(*right_arm->getCurrentState());
  const robot_state::JointModelGroup *joint_model_group =right_start_state.getJointModelGroup(right_arm->getName());
  right_start_state.setJointGroupPositions(joint_model_group, right_arm_msg_joint.position);
  right_arm->setStartState(right_start_state);

  //Updating the left arm: it retrieves the current position and sets it as the start state
  robot_state::RobotState left_start_state(*left_arm->getCurrentState());
  const robot_state::JointModelGroup *left_joint_model_group =left_start_state.getJointModelGroup(left_arm->getName());
  left_start_state.setJointGroupPositions(left_joint_model_group, left_arm_msg_joint.position);
  left_arm->setStartState(left_start_state);

}


/**
 * \brief: prints the current position of an object
 * \param po: the position of the object I want to print
 * \retval: none
 */
void stampa_Pose(Pose po)
{
  cout<<"Position"<<endl<<"X:"<<po.position.x<<endl<<"Y:"<<po.position.y<<endl<<"Z:"<<po.position.z<<endl;
  cout<<endl<<"Orientation"<<endl<<"X:"<<po.orientation.x<<endl<<"Y:"<<po.orientation.y<<endl<<"Z:"<<po.orientation.z<<endl<<"W:"<<po.orientation.w<<endl;
  tf::Quaternion q(
        po.orientation.x,
        po.orientation.y,
        po.orientation.z,
        po.orientation.w);
  tf::Matrix3x3 m(q);
  double r0, p0, y0;
  m.getRPY(r0, p0, y0);
  cout<<"r0:"<<rad_to_grad(r0)<<" p0:"<<rad_to_grad(p0)<<" y0:"<<rad_to_grad(y0);
}

/**
 * \brief: This function updates the start state of the robot
 * \param last_trajectory : last trajectory calculated
 * \param group : which moveit group is moving
 * \retval: none
 * 
 * It sets the start configuration of the robot to the one that would be when 
 * the robot is done executing the trajectory
 */
void update_start_state_after_trajectory_execution(moveit_msgs::RobotTrajectory last_trajectory,MoveGroupInterface *group){
  // I retrieve the current state of the robot
  printf("updating state \n");
  robot_state::RobotState start_state(*group->getCurrentState());
  const robot_state::JointModelGroup *joint_model_group =start_state.getJointModelGroup(group->getName());
  // I set the new start state as the one after the trajectory passed as input starting from the previous start state
  start_state.setJointGroupPositions(joint_model_group, last_trajectory.joint_trajectory.points[last_trajectory.joint_trajectory.points.size()-1].positions);
  // I set the start state for the moveit group
  group->setStartState(start_state);
  printf("state updated \n");
}

/**
 * \brief: This function is called when a server request arrives
 * \param req : the command request
 * \param res : the command response
 * \retval: true
 * 
 * The server callback, it calls the move_block function that actually 
 * implements the movement after setting the correct parameters
 */
bool server_callback(controller_baxter::command::Request &req,controller_baxter::command::Response &res){
  
  res.ok=false;	

  // It checks which arm is requested to move
  MoveGroupInterface *arm_to_move;
  if(req.arm=="left") arm_to_move=left_arm;
  if(req.arm=="right") arm_to_move=right_arm;
  // It calls the function to move a block to a desired position
  res.ok=move_block(req.cube,arm_to_move,req.pos);
 
  
  return true;
}

/**
 * \brief: initializes the joint positions
 * \param None
 * \retval: None
 */
void initialize_joints(){
left_arm_msg_joint.position.push_back(-0.52);
left_arm_msg_joint.position.push_back(-1.22);
left_arm_msg_joint.position.push_back(0);
left_arm_msg_joint.position.push_back(1.72);
left_arm_msg_joint.position.push_back(0);
left_arm_msg_joint.position.push_back(0.75);
left_arm_msg_joint.position.push_back(0);

right_arm_msg_joint.position.push_back(0.52);
right_arm_msg_joint.position.push_back(-1.22);
right_arm_msg_joint.position.push_back(0);
right_arm_msg_joint.position.push_back(1.72);
right_arm_msg_joint.position.push_back(0);
right_arm_msg_joint.position.push_back(0.75);
right_arm_msg_joint.position.push_back(0);
}

/*
// DA RIVEDERE------------------------------------------------
void add_block(){

 

  ROS_INFO_NAMED("tutorial", "Tryng to add block");

  moveit_msgs::CollisionObject collision_object;

  collision_object.header.frame_id = left_arm->getPlanningFrame();

  collision_object.id = "box1";

 

  shape_msgs::SolidPrimitive primitive;

  primitive.type = primitive.BOX;

  primitive.dimensions.resize(3);

  primitive.dimensions[primitive.BOX_X] = 2;

  primitive.dimensions[primitive.BOX_Y] = 2;

  primitive.dimensions[primitive.BOX_Z] = 2;

 

  geometry_msgs::Pose box_pose=C_blue;

  collision_object.primitives.push_back(primitive);

  collision_object.primitive_poses.push_back(box_pose);

  collision_object.operation = collision_object.ADD;

 

  std::vector<moveit_msgs::CollisionObject> collision_objects;

  collision_objects.push_back(collision_object);

 

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  planning_scene_interface.addCollisionObjects(collision_objects);

 

 

  ROS_INFO_NAMED("tutorial", "Add an object into the world");

}
*/
