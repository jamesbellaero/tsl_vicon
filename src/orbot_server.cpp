/*
DESCRIPTION: 
Subscribes to the current and target locations of the orbot and sends the delta to the orbot.

FUNCTIONS:
void 	messageCallbackVicon(geometry_msgs::TransformStamped)
	Subscribes to the orbot object on vicon at /vicon/orbot/orbot and assigns the pose to loc and att
void	messageCallbackTarget(geometry_msgs::TransformStamped)
	Subscribes to the target on /orbot_server/target and assigns the target pose to tarLoc and tarAtt
int 	main(int, char**)
	Starts the subscribers and publisher, publishes delta between target and current pose to the /orbot_server/orbot_delta

NOTES:
	Would like to move this functionality into the orbot client eventually, since the client should be able to subscribe
	to both on its own. Only currently using this for debugging/separation of tasks

Last edited by James Bell on 1/9/18
*/

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include "quaternion.h"


ros::Publisher pub;

ros::Subscriber subVicon, subTarget;
Vec3 tarLoc;
Vec3 tarAtt;
Vec3 loc;
Vec3 att;	
void messageCallbackVicon( geometry_msgs::TransformStamped t){
  //const geometry_msgs::TransformStamped *t = &transform;
  std::string a =  t.header.frame_id;//currently unused

  //defined in quaternion.h
  
			
  Vec4 quat;

  loc.v[0] = t.transform.translation.x;
  loc.v[1] = t.transform.translation.y;
  loc.v[2] = t.transform.translation.z;
  quat.v[0] = t.transform.rotation.w;
  quat.v[1] = t.transform.rotation.x;
  quat.v[2] = t.transform.rotation.y;
  quat.v[3] = t.transform.rotation.z;
  att = Quat2RPY(quat);


}
void messageCallbackTarget( geometry_msgs::TransformStamped t){
	std::string a =  t.header.frame_id;//currently unused
  Vec4 quat;

	tarLoc.v[0] = t.transform.translation.x;
	tarLoc.v[1] = t.transform.translation.y;
	tarLoc.v[2] = t.transform.translation.z;
	quat.v[0] = t.transform.rotation.w;
	quat.v[1] = t.transform.rotation.x;
	quat.v[2] = t.transform.rotation.y;
	quat.v[3] = t.transform.rotation.z;
	tarAtt = Quat2RPY(quat);
}

int main(int argc, char** argv){
	ros::init(argc,argv,"orbot_server");
	ros::NodeHandle nh;
	pub=nh.advertise<geometry_msgs::Vector3>("orbot_server/orbot_delta",1000);
	subVicon=nh.subscribe("/vicon/orbot/orbot",1000,messageCallbackVicon);
	subTarget=nh.subscribe("/orbot_server/target",1000,messageCallbackTarget);
	
	ros::Rate loop_rate(10);
	while(ros::ok()){
		//Compute and send deltas between target and current position, angle
		float dx = (float)(tarLoc.v[0] - loc.v[0]);
	  float dy = (float)(tarLoc.v[1] - loc.v[1]);
	  float dTheta  = (float)(tarAtt.v[2]-att.v[2]);
    geometry_msgs::Vector3 msg;
	  msg.x=dx;
	  msg.y=dy;
	  msg.z=dTheta;
	  pub.publish(msg);
	  ros::spinOnce();
	  loop_rate.sleep();
	}
	return 0;
}






