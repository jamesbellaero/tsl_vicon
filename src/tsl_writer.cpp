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

#include <iostream>
#include <fstream>

#include "quaternion.h"


ros::Publisher pub;

ros::Subscriber subCygnus, subCamera;
ros::Time tCyg, tCam;
Vec3 locCyg,locCam;
Vec4 quatCyg,quatCam;
DCM dcmCyg, dcmCam, dcmCam2Cyg;
void messageCallbackCygnus( geometry_msgs::TransformStamped t){
  //const geometry_msgs::TransformStamped *t = &transform;
  std::string a =  t.header.frame_id;//currently unused
  tCyg = t.header.stamp;
  //defined in quaternion.h

  locCyg.v[0] = t.transform.translation.x;
  locCyg.v[1] = t.transform.translation.y;
  locCyg.v[2] = t.transform.translation.z;
  quatCyg.v[0] = t.transform.rotation.w;
  quatCyg.v[1] = t.transform.rotation.x;
  quatCyg.v[2] = t.transform.rotation.y;
  quatCyg.v[3] = t.transform.rotation.z;
  // att = Quat2RPY(quat);


}
void messageCallbackCamera(geometry_msgs::TransformStamped t){
  //const geometry_msgs::TransformStamped *t = &transform;
  std::string a =  t.header.frame_id;//currently unused
  tCam = t.header.stamp;
  //defined in quaternion.h

  locCam.v[0] = t.transform.translation.x;
  locCam.v[1] = t.transform.translation.y;
  locCam.v[2] = t.transform.translation.z;
  quatCam.v[0] = t.transform.rotation.w;
  quatCam.v[1] = t.transform.rotation.x;
  quatCam.v[2] = t.transform.rotation.y;
  quatCam.v[3] = t.transform.rotation.z;

}

int main(int argc, char** argv){
	ros::init(argc,argv,"tsl_vicon");
	ros::NodeHandle nh;
  subCygnus=nh.subscribe("/vicon/cygnus_tsl/cygnus_tsl/",1000,messageCallbackCygnus);
  subCamera=nh.subscribe("/vicon/camera_tsl/camera_tsl",1000,messageCallbackCamera);

  std::ofstream out;
  out.open("/home/James/vicon_out.txt",std::ofstream::out);
  out << "";//clear file
  out.close();
	
	ros::Rate loop_rate(10);
	while(ros::ok()){
    if(tCyg.toNSec()!=0){
      
      dcmCyg = Quat2DCM(quatCyg);
      dcmCam = Quat2DCM(quatCam);
      dcmCam2Cyg = getRelativeRotation(dcmCam,dcmCyg);
      out.open("/home/James/vicon_out.txt",std::ofstream::out | std::ofstream::app);
      out << tCyg.toNSec()<<"\t"<<(double)(locCyg.v[0])<<"\t"<<(double)(locCyg.v[1])<<"\t"<<(double)(locCyg.v[2]);
      for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
          out<<"\t"<<(double)(dcmCam2Cyg.v[i][j]);
        }
      }
      out << "\n";
      out.close();
      std::cout<<"Wrote to file\n";
    }

	  ros::spinOnce();
	  loop_rate.sleep();
	}
	return 0;
}






