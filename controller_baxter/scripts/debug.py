#! /usr/bin/env python

import rospy
from controller_baxter.srv import at_home, at_homeRequest, at_homeResponse
from controller_baxter.srv import command, commandRequest, commandResponse
from human_baxter_collaboration.msg import UnityTf
from geometry_msgs.msg import Point, PoseStamped, Pose

import time
import math

srv_controller = None
srv_baxter_home = None

def loop() :
	global srv_controller, srv_baxter_home
	at_home_req=at_homeRequest()
	at_home_req.arm="left"
	at_home_res=at_homeResponse()
	at_home_res.at_home=True
	task = commandRequest()
	command_res=commandResponse()
	task.arm = "left"
	task.pos = "center"
	task.cube="C"
	
	
	while True : 
		rospy.loginfo("send command")
		command_res.ok=False
		while not(command_res.ok):
			command_res=srv_controller(task)
		
		while (at_home_res.at_home) :	
			at_home_res=srv_baxter_home(at_home_req)
			rospy.loginfo("still at home")
			time.sleep(1)
		
		while not (at_home_res.at_home) :	
			at_home_res=srv_baxter_home(at_home_req)
			rospy.loginfo("still not at home")
			time.sleep(1)	

def main():
    global srv_controller, msg_unity_tf, srv_baxter_home
    
    # node init
    rospy.init_node( "debug" )
    
    # service: controller_server
    srv_controller = rospy.ServiceProxy( "controller_server", command )
    rospy.loginfo( "service:controller ... OK!" )
    
    # service: baxter_at_home_server
    srv_baxter_home = rospy.ServiceProxy( "baxter_at_home_server", at_home )
    rospy.loginfo( "service: baxter at home ... OK!" )
    
    # ciclo di funzionamento
    rospy.loginfo( "starting loop" )
    loop()



if __name__ == "__main__":
    main()
