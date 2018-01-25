#include<iostream>
#include<cstdlib>
#include<cmath>
#include<memory>
#include<vector>
#include<math.h>
#include<pthread.h>
extern "C" {
	#include "extApi.h"
	}
//#include<extApi.h>
#include<extApiPlatform.h>
#include<v_repConst.h>
#include<string>
#include<Eigen/Eigen>
#include<Jacobian_Pseudoinverse.h>


Eigen::Matrix<int,5,1> get_Handles(int clientID)
{
	Eigen::Matrix<int,5,1> object_handles;
	int object;
	simxGetObjectHandle(clientID,"Needle_L1",&object,simx_opmode_blocking);
	object_handles(0,0) = object;
	simxGetObjectHandle(clientID,"Needle_L2",&object,simx_opmode_blocking);
	object_handles(1,0) = object;
	simxGetObjectHandle(clientID,"Needle_L3",&object,simx_opmode_blocking);
        object_handles(2,0) = object;
	simxGetObjectHandle(clientID,"Needle_L4",&object,simx_opmode_blocking);
        object_handles(3,0) = object;
	simxGetObjectHandle(clientID,"Needle_L5",&object,simx_opmode_blocking);
        object_handles(4,0) = object;
	
	return object_handles;
}

Eigen::Matrix<float,5,1> get_joint_positions(int clientID,bool first_call)
{
	Eigen::Matrix<int,5,1> object_handles = get_Handles(clientID);
	Eigen::Matrix<float,5,1> position;
	for(int i = 0; i< 5; i++)
	{
		float value;
		if(first_call)
		{
			simxGetJointPosition(clientID,object_handles(i,0),&value,simx_opmode_streaming);
		}
		simxGetJointPosition(clientID,object_handles(i,0),&value,simx_opmode_buffer);
		position(i,0) = value;
	}
	return position;
}

void set_joint_positions(int clientID,Eigen::Matrix<float,5,1> new_position)
{
	Eigen::Matrix<int,5,1> object_handles = get_Handles(clientID);
	simxPauseCommunication(clientID,1);
	
	for(int i = 0; i < 5; i++)
	{
		int r = simxSetJointTargetPosition(clientID,object_handles(i,0),new_position(i,0),simx_opmode_oneshot);
		if(r > 1)
		{
		std::cout<<"New Position was not written: return: "<<r<<std::endl;
		}
	}

	simxPauseCommunication(clientID,0);	
} 

 
void drawSomething(int clientID, Eigen::VectorXf vx, Eigen::VectorXf vy, Eigen::VectorXf vz, Eigen::Matrix<float,5,1> current_position)
{
        for (int i = 1; i < vx.size(); i++)
        {
                current_position = get_joint_positions(clientID,false);
                Eigen::Matrix<float,5,1> add;
		add<<0.0152,-0.0103,-0.0038,0.0038,0.0;
                Eigen::Matrix<float,5,1> calculation_position = current_position+add;

                //Compute the step values given to Jacobian_pseudoinverse
                Eigen::Matrix<float,6,1> param;
		param<<vx[i]-vx[i-1],vy[i]-vy[i-1],vz[i]-vz[i-1],0,0,0;
		//std::cout<<"Parameters given to Jacobian"<<param<<std::endl;
                Eigen::Matrix<float,5,1> delta_L = Jacobian_pseudoinverse(calculation_position,param); 
                set_joint_positions(clientID,(current_position+delta_L));
		//std::cout<<"delta_L "<<delta_L<<std::endl;
		//std::cout<<"Needle should perform a move "<<std::endl;
        }
}
//This should one day draw a square right now it draws a line
//a should be the length of one side of the square
/*void drawSqare(int clientID, float a)
{
	Eigen::Matrix<float,5,1> current_position = get_joint_positions(clientID,false);
        Eigen::Matrix<float,5,1> n;
        n<<0,0,0,0,0;
        set_joint_positions(clientID,current_position+n);

        int number_sampling_points = 10;
        //Initialize the vectors (using std vector actually using eigen vector would be nicer ..)
        Eigen::VectorXd vz (number_sampling_points); // change in z direction
        Eigen::VectorXd vy (number_sampling_points); // change in y direction
        Eigen::VectorXd vx (number_sampling_points); //change in x direction (acctually there is non)

        for (int i = 0; i < number_sampling_points; i++)
        {
                float value = (1/number_sampling_points)*i;//Devide the range from 0 to 1 in 500 pieces
                float vz_value =  0.005*std::sin(2*M_PI*value);
                float vy_value = 0.005*std::cos(2*M_PI*value);
        }
        std::cout<<"Initialized everything to draw the circle"<<std::endl;

}*/
void drawCircle(int clientID)
{
	Eigen::Matrix<float,5,1> current_position = get_joint_positions(clientID,false);
	Eigen::Matrix<float,5,1> n;
	n<<0,0,0,0,0;
	set_joint_positions(clientID,current_position+n);

	int number_sampling_points = 10;
	//Initialize the vectors (using std vector actually using eigen vector would be nicer ..)
	Eigen::VectorXf vz (number_sampling_points); // change in z direction
	Eigen::VectorXf vy (number_sampling_points); // change in y direction
	Eigen::VectorXf vx (number_sampling_points); //change in x direction (acctually there is non)

	for (int i = 0; i < number_sampling_points; i++)
	{
		float value = (1/number_sampling_points)*i+1;//Devide the range from 0 to 1 in 500 pieces
		vz[i] =  std::sin(2*M_PI*value);
		vy(i,0) = 0.005*std::cos(2*M_PI*value);
		vx(i,0) = 0;
		
	}
	std::cout<<"vz: "<<vz<<std::endl;
	drawSomething(clientID,vx,vy,vz,current_position);
}

int main(int argc, char* argv[])
{
	int portNb = 19999;
	//portNb = atoi(argv[1]);
	std::cout<<"Got until here, Portnumber: "<<portNb;

	int clientID = simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);
	std::cout<<"Survived simxStart() clientID:"<<clientID<<std::endl;
	if(clientID >= 0)
	{
		std::cout<<"Connected :)";

		//Handles
		int needle_tip_handle;
		int target_handle;
		int base_handle;

		//The simxGetObjectHandle writes the pointer to the handle into the third argument
		//Everything else is simular as in matlab		
		simxGetObjectHandle(clientID,"Needle_tip",&needle_tip_handle,simx_opmode_blocking);
		simxGetObjectHandle(clientID,"IK_Target",&target_handle,simx_opmode_blocking);
		simxGetObjectHandle(clientID,"Needle_base",&base_handle,simx_opmode_blocking);
		std::cout<<"Got all Handles: \n needle_tip_handle: "<<needle_tip_handle<<"\ntarget_handle: "<<target_handle<<"\nbase_handle: "<<base_handle<<std::endl;
		//positions
		Eigen::Matrix<float,5,1> current_position = get_joint_positions(clientID,true);
		std::cout<<"First current_position"<<current_position;

		//Create Pointer to 3 floats receiving the needle position
		std::shared_ptr<float> needle_position = std::make_shared<float>(3);
		simxGetObjectPosition(clientID,needle_tip_handle,base_handle,needle_position.get(),simx_opmode_streaming);
		simxGetObjectPosition(clientID,needle_tip_handle,base_handle,needle_position.get(),simx_opmode_buffer);
		
		//Create Pointer to 3 floats receiving the angles(Euler space)
		std::shared_ptr<float> angle_error = std::make_shared<float>(3);
		simxGetObjectOrientation(clientID,target_handle,needle_tip_handle,angle_error.get(),simx_opmode_buffer);
		
		//Create Pointer to 3 floats receiving the target position
		std::shared_ptr<float> target_position = std::make_shared<float>(3);
		simxGetObjectPosition(clientID,target_handle,base_handle,target_position.get(),simx_opmode_blocking);
		
		std::cout<<"Initialization done start working"<<std::endl;
		//Now we can start do something
		drawCircle(clientID);	
	simxFinish(clientID);
	}
return(0);
}

