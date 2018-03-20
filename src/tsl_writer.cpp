/*
DESCRIPTION: 
Subscribes to the Cygnus and Camera and writes the relative pose to a file. 
File output is formatted as 

Time (nanosecs), dx (meters), dy, dz, R11, R12, R13, R21, R22, R23, R31, R32, R33 \n

Note that the delta position is defined as r_cyg-r_cam
Also, the rotation matrix takes a vector from the camera frame to the cygnus frame. To reverse this, change it in quaternion.h
  by switching the transposed matrix and reversing multiplication order. Or just complain to the author of this file.

FUNCTIONS:
void 	messageCallbackCygnus(geometry_msgs::TransformStamped)
	Subscribes to the orbot object on vicon at /vicon/cygnus_tsl/cygnus_tsl and assigns the pose to locCyg, quatCyg
void	messageCallbackCamera(geometry_msgs::TransformStamped)
	Subscribes to the target on /vicon/camera_tsl/camera_tsl and assigns the pose to locCam, quatCam
int 	main(int, char**)
	Initializes the two subscribers, clears the file, then starts printing at ~9-10 Hz


Last edited by James Bell (jtb2013@gmail.com) on 3/20/18
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
  std::string a =  t.header.frame_id;//currently unused
  tCyg = t.header.stamp;


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
  std::string a =  t.header.frame_id;//currently unused
  tCam = t.header.stamp;


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
  out.open("../vicon_out.txt",std::ofstream::out);
  out << "";//clear file
  out.close();
	
	ros::Rate loop_rate(10);
	while(ros::ok()){
    if(tCyg.toNSec()!=0){
      
      dcmCyg = Quat2DCM(quatCyg);
      dcmCam = Quat2DCM(quatCam);
      dcmCam2Cyg = getRelativeRotation(dcmCam,dcmCyg);

      float dx = (float)(locCyg.v[0]-locCam.v[0]);
      float dy = (float)(locCyg.v[1]-locCam.v[1]);
      float dz = (float)(locCyg.v[2]-locCam.v[2]);
      out.open("/home/James/vicon_out.txt",std::ofstream::out | std::ofstream::app);
      out << tCyg.toNSec()<<","<<dx<<","<<dy<<","<<dz;
      for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
          out<<","<<(double)(dcmCam2Cyg.v[i][j]);
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






