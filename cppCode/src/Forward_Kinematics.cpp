#include "Forward_Kinematics.h"
#include <math.h>
#include <Eigen/Eigen>

Eigen::Matrix<float,6,1> forward_kinematics(Eigen::Matrix<float,5,1> L) 
{
		//input  variables
		float L1 = L(0,0);
		float L2 = L(1,0);
		float L3 = L(2,0);
		float L4 = L(3,0);
		float L5 = L(4,0);

		float theta1 = atan((L2 - L1) / 0.017);
		float theta2 = atan((L3 - L4) / 0.017);
		float q0 = (L2 + L1) / 2;
		float q1 = theta1;
		float q2 = (L4 + L3) / 2;
		float q3 = theta2;
		float q4 = L5;
		float q5 = 0;

		float l1 = 0.0330025;
		float l2 = 0;
		float l3 = 0.0400025;

		float d1 = 0.0111654;
		float d2 = 0.029685;
		float d3 = d1;
		float d4 = 0.0962734;

		float s1 = sin(q1);
		float s3 = sin(q3);
		float s5 = sin(q5);
		float c1 = cos(q1);
		float c3 = cos(q3);
		float c5 = cos(q5);
		
		float T00 = -s1*s5*s3;
		float T01 = c1*s3*s5-c5*s1;
		float T02 = -c1*c3;
		float T03 = l2*c1+l1*s1-c1*c3*(d4+q4)-l3*c1*s3;
		float T10 = c3*c5;
		float T11 = -c3*c5;
		float T12 = -s3;
		float T13 = d2+d3+q2-s3*(d4+q4)+l3*c3;
		float T20 = c5*s1*s3-c1*s5;
		float T21 = -c1*c5-s1*s3*s5;
		float T22 = c3*s1;
		float T23 = d1+q0+l1*c1-l2*s1+c3*s1*(d4+q4)+l3*s1*s3;

		Eigen::Matrix4f T6;

		T6 << T00,T01,T02,T03,
			T10,T11,T12,T13,
			T20,T21,T22,T23,
			0,0,0,1;

		Eigen::Matrix<float,6,1> P;
		
		P(0,0) = T6(0,3);
		P(1,0) = T6(1,3);
		P(2,0) = T6(2,3);
		P(3,0) = 0;
		P(4,0) = 0;
		P(5,0) = 0;

		return P;

}
