#include <math.h>
struct Vec4{
	float v[4]={1,0,0,0};
};

struct Vec3{
	float v[3]={0};
};

struct DCM{
	float v[3][3]={{1,0,0},{0,1,0},{0,0,1}};
};

Vec3 Quat2RPY(Vec4 q){
	float q0 = q.v[0]; //q0 = q.v[1];
	float q1 = q.v[1]; //q1 = q.v[2];
	float q2 = q.v[2]; //q2 = q.v[3];
	float q3 = q.v[3]; //q3 = q.v[0];
	Vec3 RPY;

	RPY.v[0] = atan2(2*(q0*q1 + q2*q3) , 1 - 2*(q1*q1 + q2*q2) );	//Roll
	RPY.v[1] = asin(2*(q0*q2 - q3*q1));				//Pitch
	RPY.v[2] = atan2(2*(q0*q3 + q1*q2),1 - 2*(q2*q2 + q3*q3) );	//Yaw

	return RPY;
}
DCM Quat2DCM(Vec4 q){
	/*converts quaternion to a Direction Cosine Matrix (rotation matrix)*/
	DCM M;
	float q0 = q.v[0]; 
	float q1 = q.v[1]; 
	float q2 = q.v[2]; 
	float q3 = q.v[3];
	M.v[0][0]=q0*q0+q1*q1-q2*q2-q3*q3;
	M.v[0][1]=2*(q1*q2-q0*q3);
	M.v[0][2]=2*(q0*q2+q1*q3);
	M.v[1][0]=2*(q1*q2+q0*q3);
	M.v[1][1]=q0*q0-q1*q1+q2*q2-q3*q3;
	M.v[1][2]=2*(q2*q3-q0*q1);
	M.v[2][0]=2*(q1*q3-q0*q2);
	M.v[2][1]=2*(q0*q1+q2*q3);
	M.v[2][2]=q0*q0-q1*q1-q2*q2+q3*q3;
	return M;
}

DCM getRelativeRotation(DCM Rri, DCM Rbi){
	/*Returns rotation from reference to body
	first matrix is rotation from inertial to reference
	second matrix is rotation from inertial to body*/

	DCM Rir;//transpose, could just change how it's multiplied below but that's unintuitive
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			Rir.v[i][j]=Rri.v[j][i];
		}
	}
	DCM Rbr;
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			for(int k=0;k<3;k++){
				Rbr.v[i][j]+=Rbi.v[i][k]*Rir.v[k][j];
			}
		}
	}
	return Rbr;


}
