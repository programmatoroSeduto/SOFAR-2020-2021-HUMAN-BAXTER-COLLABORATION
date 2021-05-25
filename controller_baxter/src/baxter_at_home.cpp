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
sensor_msgs::JointState messaggio_joint,left_harm_msg_joint,right_harm_msg_joint;

//SUBSCRIBER CALLBACKS
void joints_callback(const sensor_msgs::JointState& msg );

//SERVICES
bool server_callback(controller_baxter::at_home::Request &req,controller_baxter::at_home::Response &res);

//FUNCTIONS
bool very_close(double A ,double B);

int main(int argc, char** argv)
{
// Initialize the ROS Node
ros::init(argc, argv, "baxter_at_home");
ros::NodeHandle n;
ros::Subscriber joint_state_sub = n.subscribe("baxter_joint_states", 10,joints_callback);
ros::ServiceServer baxter_at_home_server = n.advertiseService("baxter_at_home_server",server_callback);

while(ros::ok()){
 ros::spinOnce();
 }
}

void joints_callback(const sensor_msgs::JointState& msg)
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

bool server_callback(controller_baxter::at_home::Request &req,controller_baxter::at_home::Response &res){
	
 res.ok=false;	
 sensor_msgs::JointState messaggio_joint;
 if(req.arm=="right") messaggio_joint=right_harm_msg_joint;
 if(req.arm=="left")  messaggio_joint=left_harm_msg_joint;
 if(req.arm!="left"&& req.arm!="right") return false;
 
 cout<<"joint0: "<<messaggio_joint.position[0]<<endl;
 cout<<"joint1: "<<messaggio_joint.position[1]<<endl;
 cout<<"joint2: "<<messaggio_joint.position[2]<<endl;
 cout<<"joint3: "<<messaggio_joint.position[3]<<endl;
 cout<<"joint4: "<<messaggio_joint.position[4]<<endl;
 cout<<"joint5: "<<messaggio_joint.position[5]<<endl;
 cout<<"joint6: "<<messaggio_joint.position[6]<<endl;
 
 if((very_close(messaggio_joint.position[0],0.52)||very_close(messaggio_joint.position[0],-0.52))&& 
 very_close(messaggio_joint.position[1],-1.22)&& 
 very_close(messaggio_joint.position[2],0) && 
 very_close(messaggio_joint.position[3],1.72) && 
 very_close(messaggio_joint.position[4],0) && 
 very_close(messaggio_joint.position[5],0.75) && 
 very_close(messaggio_joint.position[6],0))
	{res.ok=true;}
	
 return true;	
}
bool very_close(double A ,double B){
if(A<B+00.1 && A>B-0.01)return true;
return false;		
}

