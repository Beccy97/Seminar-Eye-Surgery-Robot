#include "Jacobian_Pseudoinverse.h"
#include <math.h>
#include <Eigen/Eigen>


Eigen::Matrix<float,5,1> Jacobian_pseudoinverse(Eigen::Matrix<float,5,1> current_Position, Eigen::Matrix<float,6,1> step) {   //We need a constant expression for the array Q, which is problematic for the variable step as a parameter 
	   //change this value if we need different timesteps

		//Denavit-Hartenberg-parameters

		Eigen::Matrix<float,5,1> L;
		float l1 = 0.071;
		float l2 = 0.0335;
		float l3 = -0.023;
		float l4 = 0.071;
		float l5 = 0.028;
		float l6 = 0.097;

		current_Position[0] = -current_Position[0];
		current_Position[2] = -current_Position[2];

		float q0 = (current_Position[1] + current_Position[0]) / 2;
		float q1 = atan((current_Position[1] - current_Position[0]) / 0.017);
		float q2 = (current_Position[3] + current_Position[2]) / 2;
		float q3 = atan((current_Position[3] - current_Position[2]) / 0.017);
		float q4 = current_Position[4];


		float s1 = sin(q1);
		float c1 = cos(q1);
		float s3 = sin(q3);
		float c3 = cos(q3);

		float J01 = l2*c1+l6*c3*c1+l5*s3+q4*c3*s1;
		float J03 = c1*(l6*s3-l5*c3 + q4*s3);
		float J13 = -l6 * c3 -l5*s3 -q4*c3;
		float J21 = l6*c1*c3-l2*s1+l5*c1*s3 + q4*c1*c3;
		float J23 = -s1*(l6*s3 -l5*c3 + q4*s3);
		

		Eigen::Matrix<float,6,6> J;
		J<< 0,J01,0,J03,-c1*c3, 0,
			0,0, 1, J13, -s3,    0,
			1,J21, 0, J23, c3*s1, 0,
			0,0,0,s1,0,-c1*c3,
			0,1,0,0,0,-s3,
			0,0,0,c1,0, c3*s1 ;

		//get size of the matrix , Problem liegt bei step

		Eigen::Matrix<float,6,1> Q;
		Q = J.inverse()*step;//Maybe find a way to speed this up...

		L(1,0) = (0.017/2)*tan(Q(1,0)) + Q(0,0);   //We try to access the entries along the first column 
		L(0,0) = Q(0,0) - (0.017 / 2)*tan(Q(1,0));
		L(3,0) = (0.017 / 2)*tan(Q(3,0)) + Q(2,0);
		L(2,0) = Q(2,0)- (0.017 / 2) *tan(Q(4,0));
		L(4,0) = Q(4,0);
		L(0,0)= -1*L(0,0);
		L(2,0) = -1*L(2,0);


		return L;
	}

