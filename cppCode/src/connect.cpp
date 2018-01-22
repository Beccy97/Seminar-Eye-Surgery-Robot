#include<iostream>
#include<cstdlib>
#include<cmath>
#include<memory>
#include<vector>
#include<math.h>
#include<extApi.h>
#include<extApiPlatform.h>

#define NON_MATLAB_PARSING
#define MAX_EXT_API_CONNECTIONS = 255

std::vector<float> get_joint_positions(int clientID,bool first_call)
{
	std::vector<int> object_handles;
	int object;
	simxGetObjectHandle(clientID,"Needle_L1",&object,simx_opmode_blocking);
	object_handles.push_back(object);
	simxGetObjectHandle(clientID,"Needle_L2",&object,simx_opmode_blocking);
	object_handles.push_back(object);
	simxGetObjectHandle(clientID,"Needle_L3",&object,simx_opmode_blocking);
        object_handles.push_back(object);
	simxGetObjectHandle(clientID,"Needle_L4",&object,simx_opmode_blocking);
        object_handles.push_back(object);
	simxGetObjectHandle(clientID,"Needle_L5",&object,simx_opmode_blocking);
        object_handles.push_back(object);

	std::vector<float> position;
	for(int i = 0; i< 5; i++)
	{
		float value;
		if(first_call)
		{
			simxGetJointPosition(clientID,object_handles.at(i),&value,simx_opmode_streaming);
		}
		simxGetJointPosition(clientID,object_handles.at(i),&value,simx_opmode_buffer);
		position.push_back(value);
	}
	return position;
}

//To-Do Translate forward_kinematics
//To-do Translate Jacobian_pseudoinverse
//To-Do Translate set_joint_positions

void drawCircle(int clientID, std::vector<float> current_position)
{
	//Initialize the vectors (using std vector actually using eigen vector would be nicer ..)
	std::vector<float> vz; // change in z direction
	std::vector<float> vy; // change in y direction
	float vx = 0; //change in x direction (acctually there is non

	for (int i = 0; i < 500; i++)
	{
		float value = (1/500)*i;//Devide the range from 0 to 1 in 500 pieces
		float vz_value =  0.005*std::sin(2*M_PI*value);
		float vy_value = 0.005*std::cos(2*M_PI*value);
	}

	for (int i = 0; i < 500; i++)
	{
		current_position = get_joint_positions(clientID,false);
		//Todo Continue method		
	}
}
int main(int argc, char* argv[])
{
	int portNb = 0;
	portNb = atoi(argv[1]);

	int clientID = simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);
	if(clientID > -1)
	{
		std::cout<<"Connected :)";

		//Handles
		int needle_tip_handle;
		int target_handle;
		int base_handle;
		//This Shared Ptr is a safer "Container" for a pointer it works like a pointer !!

		//The simxGetObjectHandle writes the pointer to the handle into the third argument
		//Everything else is simular as in matlab		
		simxGetObjectHandle(clientID,"Needle_tip",&needle_tip_handle,simx_opmode_blocking);
		simxGetObjectHandle(clientID,"IK_Target",&target_handle,simx_opmode_blocking);
		simxGetObjectHandle(clientID,"Needle_base",&base_handle,simx_opmode_blocking);

		//positions
		std::vector<float> current_position = get_joint_positions(clientID,true);
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
		
		//Now we can start do something
			
	simxFinish(clientID);
	}
return(0);
}

