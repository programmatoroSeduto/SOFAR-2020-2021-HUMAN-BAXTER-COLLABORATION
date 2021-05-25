#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <human_baxter_collaboration/BaxterTrajectory.h>
#include <human_baxter_collaboration/UnityTf.h>
#include <controller_baxter/command.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/RobotState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "iostream"
#include <tf/tf.h>
#include <math.h>
//NAMESPACES
using namespace std;
using namespace moveit;
using namespace planning_interface;
using namespace geometry_msgs;

//GLOBAL VARIABLES
ros::Publisher trajectory_pub;
sensor_msgs::JointState right_harm_msg_joint,left_harm_msg_joint,both_harm_msg_joint;
Pose hand_r_pose,hand_l_pose,F_pose;
robot_model::RobotModelPtr kinematic_model_right,kinematic_model_left;
robot_state::RobotStatePtr kinematic_state_right,kinematic_state_left;
const robot_state::JointModelGroup *joint_model_group_right,*joint_model_group_left;
Eigen::Affine3d end_effector_state_right,end_effector_state_left;
//SUBSCRIBER CALLBACKS
void tf_callback(const human_baxter_collaboration::UnityTf& msg);
void update_joints_callback(const sensor_msgs::JointState& msg );
void check_collision();
//SERVICES

//FUNCTIONS
void update_start_state_from_callback();
void stampa_Pose(Pose po);
double rad_to_grad(double rad);


//
int main(int argc, char** argv)
{
// Initialize the ROS Node
ros::init(argc, argv, "collision_avoidance");
ros::NodeHandle n;
ros::Subscriber tf_sub = n.subscribe("unity_tf", 10, tf_callback);
ros::Subscriber joint_state_sub = n.subscribe("baxter_joint_states", 10, update_joints_callback);
static ros::AsyncSpinner spinner(1);
spinner.start();
ros::WallDuration(1.0).sleep();


robot_model_loader::RobotModelLoader robot_model_loader_right("robot_description");
kinematic_model_right = robot_model_loader_right.getModel();
robot_state::RobotStatePtr kinematic_state_temp_right(new robot_state::RobotState(kinematic_model_right));
kinematic_state_right=kinematic_state_temp_right;
kinematic_state_right->setToDefaultValues();
joint_model_group_right = kinematic_model_right->getJointModelGroup("right_arm");

robot_model_loader::RobotModelLoader robot_model_loader_left("robot_description");
kinematic_model_left = robot_model_loader_left.getModel();
robot_state::RobotStatePtr kinematic_state_temp_left(new robot_state::RobotState(kinematic_model_left));
kinematic_state_left=kinematic_state_temp_left;
kinematic_state_left->setToDefaultValues();
joint_model_group_left = kinematic_model_left->getJointModelGroup("left_arm");




while(ros::ok()){
ros::spinOnce();
update_start_state_from_callback();
check_collision();
}

return 0;
}
void check_collision(){
double diff_right_x,diff_right_y,distance_on_right,diff_left_x,diff_left_y,distance_on_left;//distanza tra braccio destro e gripper destro; distanza tra braccio sx e gripper sx
diff_right_x=(end_effector_state_right.translation().x() - hand_l_pose.position.x);
diff_right_y=(end_effector_state_right.translation().y() - hand_l_pose.position.y);
distance_on_right=sqrt(pow(diff_right_x,2)+pow(diff_right_y,2));

diff_left_x=(end_effector_state_left.translation().x() - hand_r_pose.position.x);
diff_left_y=(end_effector_state_left.translation().y() - hand_r_pose.position.y);
distance_on_left=sqrt(pow(diff_left_x,2)+pow(diff_left_y,2));


if(distance_on_right<0.2 || distance_on_left<0.2)
  ROS_INFO("Actual_distances:\nRight: %f \nLeft: %f",distance_on_right,distance_on_left);


}
void tf_callback(const human_baxter_collaboration::UnityTf& msg)
{

  for(unsigned long i=0;i<msg.frames.size();i++){
    if(msg.frames[i].header.frame_id=="hand_r"){

      hand_r_pose=msg.frames[i].pose;

    }
    if(msg.frames[i].header.frame_id=="hand_l"){

      hand_l_pose=msg.frames[i].pose;

    }
    if(msg.frames[i].header.frame_id=="F"){

      F_pose=msg.frames[i].pose;

    }

  }

}
void update_joints_callback(const sensor_msgs::JointState& msg)
{
 right_harm_msg_joint=msg;
 right_harm_msg_joint.name.erase(right_harm_msg_joint.name.begin());
 right_harm_msg_joint.name.erase(right_harm_msg_joint.name.begin()+7,right_harm_msg_joint.name.begin()+18);
 right_harm_msg_joint.position.erase(right_harm_msg_joint.position.begin());
 right_harm_msg_joint.position.erase(right_harm_msg_joint.position.begin()+7,right_harm_msg_joint.position.begin()+18);

 left_harm_msg_joint=msg;
 left_harm_msg_joint.name.erase(left_harm_msg_joint.name.begin());
 left_harm_msg_joint.name.erase(left_harm_msg_joint.name.begin(),left_harm_msg_joint.name.begin()+7);
 left_harm_msg_joint.name.erase(left_harm_msg_joint.name.begin()+7,left_harm_msg_joint.name.begin()+11);
 left_harm_msg_joint.position.erase(left_harm_msg_joint.position.begin());
 left_harm_msg_joint.position.erase(left_harm_msg_joint.position.begin(),left_harm_msg_joint.position.begin()+7);
 left_harm_msg_joint.position.erase(left_harm_msg_joint.position.begin()+7,left_harm_msg_joint.position.begin()+11);

}
void update_start_state_from_callback(){

  kinematic_state_right->setJointGroupPositions(joint_model_group_right, right_harm_msg_joint.position);
  end_effector_state_right = kinematic_state_right->getGlobalLinkTransform("right_gripper");

  kinematic_state_left->setJointGroupPositions(joint_model_group_left, left_harm_msg_joint.position);
  end_effector_state_left = kinematic_state_left->getGlobalLinkTransform("left_gripper");

}
double rad_to_grad(double rad)
{
return rad*180/3.1415;
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
  cout<<"r0:"<<rad_to_grad(r0)<<" p0:"<<rad_to_grad(p0)<<" y0:"<<rad_to_grad(y0)<<endl;
}
