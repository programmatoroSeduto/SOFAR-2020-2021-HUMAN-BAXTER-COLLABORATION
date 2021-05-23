#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <human_baxter_collaboration/BaxterTrajectory.h>
#include <human_baxter_collaboration/UnityTf.h>
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
//MoveGroupInterface *left_arm;
//MoveGroupInterface *both_arms;
ros::Publisher trajectory_pub;
sensor_msgs::JointState messaggio_joint;
Pose target_block,red_box_pose,blue_box_pose,A_red,B_red,C_blue,D_red,E_blue,F_red,G_blue,H_red,I_blue,L_red,M_blue;

//SUBSCRIBER CALLBACKS
void tf_callback(const human_baxter_collaboration::UnityTf& msg);
void update_joints_callback(const sensor_msgs::JointState& msg );

//FUNCTIONS
void print_all_joints();
double rad_to_grad(double rad);
void move_to_pose(geometry_msgs::Pose pt, bool Orientamento);
void update_start_state();
void stampa_Pose(Pose po);
void move_to_waypoints(vector<Pose> waypoints);
void prova();


int main(int argc, char** argv)
{
// Initialize the ROS Node
ros::init(argc, argv, "baxter_controller");
ros::NodeHandle n;
ros::Subscriber tf_sub = n.subscribe("unity_tf", 10, tf_callback);
ros::Subscriber joint_state_sub = n.subscribe("baxter_joint_states", 10, update_joints_callback);
trajectory_pub = n.advertise<human_baxter_collaboration::BaxterTrajectory>("baxter_moveit_trajectory", 10);
static ros::AsyncSpinner spinner(1);
spinner.start();
ros::WallDuration(1.0).sleep();

MoveGroupInterface movegroup_right("right_arm");
//MoveGroupInterface movegroup_left("left_arm");
//MoveGroupInterface movegroup_both("both_arms");
right_arm=&movegroup_right;
//left_arm=&movegroup_left;
//both_arms=&movegroup_both;
vector<double> joints_right;
//vector<double> joints_left;
//vector<double> joints_both;
joints_right=right_arm->getCurrentJointValues();//NON FUNZIONA(prende tutti joint a zero)
//joints_left=left_arm->getCurrentJointValues(); //NON FUNZIONA(prende tutti joint a zero)
//joints_both=both_arms->getCurrentJointValues(); //NON FUNZIONA(prende tutti joint a zero)


ROS_INFO("size: %d",right_arm->getVariableCount());


string choice;
do{

  ros::spinOnce();

  update_start_state();

  cout<<"Inserisci una delle seguenti scelte:\n 0)chiudi\n 1)Muovi \nChoice:";
  cin>>choice;
  if(choice=="1"){

    move_to_pose(target_block,true);

  }
  if(choice=="2"){
    prova();
  }
}while(ros::ok()&& choice!="0");
return 0;
}

void prova(){
  ROS_INFO("Trying to execute a trajectory");
  vector<Pose> waypoints;
  Pose target2;
  MoveGroupInterface::Plan my_plan;
  bool success;


  target2=target_block;
  target2.position.z+=0.15;

  right_arm->setPoseTarget(target2);
  success = (right_arm->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Plan Result:%s", success ? "SUCCESS" : "FAILED");
  human_baxter_collaboration::BaxterTrajectory my_trajectory;
  my_trajectory.arm="right"; //MUOVE IL DESTRO!
  my_trajectory.trajectory.push_back(my_plan.trajectory_);


  robot_state::RobotState start_state(*right_arm->getCurrentState());
  const robot_state::JointModelGroup *joint_model_group =start_state.getJointModelGroup(right_arm->getName());
  start_state.setJointGroupPositions(joint_model_group, my_plan.trajectory_.joint_trajectory.points[my_plan.trajectory_.joint_trajectory.points.size()-1].positions);
  right_arm->setStartState(start_state);



  right_arm->setPoseTarget(target_block);
  success = (right_arm->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Plan Result:%s", success ? "SUCCESS" : "FAILED");
  my_trajectory.trajectory.push_back(my_plan.trajectory_);
//arriva al blocco




  joint_model_group =start_state.getJointModelGroup(right_arm->getName());
  start_state.setFromIK(joint_model_group, target_block);
  right_arm->setStartState(start_state);
  //setta come start il blocco



  right_arm->setPoseTarget(target2);
  success = (right_arm->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Plan Result:%s", success ? "SUCCESS" : "FAILED");
  my_trajectory.trajectory.push_back(my_plan.trajectory_);
  //arriva un po più in alto del blocco


  joint_model_group =start_state.getJointModelGroup(right_arm->getName());
  start_state.setFromIK(joint_model_group, target2);
  right_arm->setStartState(start_state);
  //setta come start il punto più in alto del blocco




  right_arm->setPoseTarget(red_box_pose);
  success = (right_arm->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Plan Result:%s", success ? "SUCCESS" : "FAILED");
  my_trajectory.trajectory.push_back(my_plan.trajectory_);
  //arriva alla red box


  trajectory_pub.publish(my_trajectory);
}

void move_to_pose(geometry_msgs::Pose pt, bool Orientamento){
  MoveGroupInterface::Plan my_plan;
  if(!Orientamento)
    right_arm->setPositionTarget(pt.position.x,pt.position.y,pt.position.z);
  else
    right_arm->setPoseTarget(pt);

  bool success = (right_arm->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Plan Result:%s", success ? "SUCCESS" : "FAILED");
  human_baxter_collaboration::BaxterTrajectory my_trajectory;
  my_trajectory.arm="right"; //MUOVE IL DESTRO!
  my_trajectory.trajectory.push_back(my_plan.trajectory_);
  trajectory_pub.publish(my_trajectory);
}
void move_to_waypoints(vector<Pose> waypoints){
/*
      MoveGroupInterface::Plan my_plan;
      human_baxter_collaboration::BaxterTrajectory my_trajectory;
      moveit_msgs::RobotTrajectory trajectory;
      right_arm->computeCartesianPath(waypoints,0.001,0,trajectory);
      //bool success = (right_arm->plan(my_plan) == MoveItErrorCode::SUCCESS);
      //ROS_INFO_NAMED("tutorial", "Plan Result:%s", success ? "SUCCESS" : "FAILED");
      my_trajectory.arm="right"; //MUOVE IL DESTRO!
      my_trajectory.trajectory.push_back(my_plan.trajectory_);
      trajectory_pub.publish(my_trajectory);
*/
}
void tf_callback(const human_baxter_collaboration::UnityTf& msg)
{

  for(unsigned long i=0;i<msg.frames.size();i++){
    if(msg.frames[i].header.frame_id=="E"){

      target_block=msg.frames[i].pose;

    }
    if(msg.frames[i].header.frame_id=="Redbox"){

      red_box_pose=msg.frames[i].pose;
      red_box_pose.position.z+=0.15;

    }

  }

}
void update_joints_callback(const sensor_msgs::JointState& msg)
{
 messaggio_joint=msg;
 messaggio_joint.name.erase(messaggio_joint.name.begin());
 messaggio_joint.name.erase(messaggio_joint.name.begin()+7,messaggio_joint.name.begin()+18);
 messaggio_joint.position.erase(messaggio_joint.position.begin());
 messaggio_joint.position.erase(messaggio_joint.position.begin()+7,messaggio_joint.position.begin()+18);

}
void print_all_joints()
{
vector<double> joint_group_positions;
joint_group_positions=right_arm->getCurrentJointValues();
cout<<endl<<"Giunti:"<<endl;
for(int i=0;i<joint_group_positions.size();i++){
cout<<i<<":"<<joint_group_positions[i]<<" xxx "<<rad_to_grad(joint_group_positions[i])<<endl;
}
}
double rad_to_grad(double rad)
{
return rad*180/3.1415;
}
void update_start_state(){

  robot_state::RobotState start_state(*right_arm->getCurrentState());
  const robot_state::JointModelGroup *joint_model_group =start_state.getJointModelGroup(right_arm->getName());
  start_state.setJointGroupPositions(joint_model_group, messaggio_joint.position);
  right_arm->setStartState(start_state);

}
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
