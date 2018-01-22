#include "Header1.h"
#include <math.h>
#include <array>


class Jacobian_Pseudoinverse {

	double* Jacobian_Pseudoinverse::matrix(double current_Position[], int step) {   //We need a constant expression for the array Q, which is problematic for the variable step as a parameter 
	   //change this value if we need different timesteps

		double L[5];
		double l1 = 0.071;
		double l2 = 0.0335;
		double l3 = -0.023;
		double l4 = 0.071;
		double l5 = 0.028;
		double l6 = 0.097;

		current_Position[1] = -current_Position[1];
		current_Position[3] = -current_Position[3];

		double q0 = (current_Position[2] + current_Position[1]) / 2;
		double q1 = atan((current_Position[2] - current_Position[1]) / 0.017);
		double q2 = (current_Position[4] + current_Position[3]) / 2;
		double q3 = atan((current_Position[4] - current_Position[3]) / 0.017);
		double q4 = current_Position[5];


		double s1 = sin(q1);
		double c1 = cos(q1);
		double s3 = sin(q3);
		double c3 = cos(q3);

		double J[6][6] = { {0, l2*c1 + l6*c3*s1 + l5*s3 + q4*c3*s1, 0, c1*(l6*s3 - l5*c3 + q4*s3), -c1*c3, 0},
					   {0,                             0, 1, -l6 * c3 - l5*s3 - q4*c3, -s3,    0},
					   {1, l6*c1*c3 - l2*s1 + l5*c1*s3 + q4*c1*c3, 0, -s1*(l6*s3 - l5*c3 + q4*s3), c3*s1, 0},
					   {0,							0	,  0   ,s1      ,      0      ,-c1*c3},
					   {0,1,0,0,0,-s3},
					   {0,0,0,c1,0, c3*s1} };

		//get size of the matrix , Problem liegt bei step

		double Q[6][6];
		std::copy(&J[0][0], &J[0][0] + 6*6, &Q[0][0]);   //Q copies all the values of J
		for (int y = 0; y < 6; y++) {
			for (int x = 0; x < 6; x++) {
				Q[x][y]/step;  //then step will be divided by all entries
			}
		}

		L[2] = (0.017 / 2)*tan(Q[0][2]) + Q[0][1];   //We try to access the entries along the first column 
		L[1] = Q[0][1] - (0.017 / 2)*tan(Q[0][2]);
		L[4] = (0.017 / 2)*tan(Q[0][4]) + Q[0][3];
		L[3] = Q[0][3] - (0.017 / 2) *tan(Q[0][4]);
		L[5] = Q[0][5];
		L[1] = -L[1];
		L[3] = -L[3];


		return L;
	}


	/*double Jacobian_Pseudoinverse::getSize(double array[6][6]) {
	double size = sizeof(array) / sizeof(array[0]);
	return size;
	}*/
};