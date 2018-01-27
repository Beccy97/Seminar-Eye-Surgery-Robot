#define _USE_MATH_DEFINES 
#include<iostream>
#include<cstdlib>
#include<chrono>
#include<cmath>
#include<memory>
#include<vector>
#include<math.h>
#include<chrono>
#include<thread>
extern "C" {
	#include "extApi.h"
	}
#include<extApiPlatform.h>
#include<v_repConst.h>
#include<string>
#include<Eigen/Eigen>
#include<Jacobian_Pseudoinverse.h>

//get the Object needles from v_rep
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

//get the joit Positions from v_rep
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

//set the computed new joint positions in v_rep
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


//This draws the values from vx,vy,vz in v_rep
//The number of sampling points is assumed to b the length of the vectors, therefore all of the 3 vectors need to have the same length
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
                Eigen::Matrix<float,5,1> delta_L = Jacobian_pseudoinverse(calculation_position,param); 
                set_joint_positions(clientID,(current_position+delta_L));
        }
}

//Draw a sqare in the x,y plane in v_rep
//The legth denotes the length of the diagonal of the square
void drawSquare(int clientID, float length)
{
	Eigen::Matrix<float,5,1> current_position = get_joint_positions(clientID,false);
        Eigen::Matrix<float,5,1> n;
        n<<0,0,0,0,0;
        set_joint_positions(clientID,current_position+n);

        int number_sampling_points = 50;//Number of sampling points per edge

        Eigen::VectorXf vx (number_sampling_points); // change in x direction
        Eigen::VectorXf vy (number_sampling_points); // change in y direction
        Eigen::VectorXf vz (number_sampling_points); //change in z direction 
	Eigen::VectorXf nvx (number_sampling_points);//vx the other way around

        for (int i = 0; i < number_sampling_points; i++)
        {
                float value = (length/number_sampling_points)*(i+1);//Devide the range from 0 to length in number_sampling_points pieces
                vx(i,0) =  value;
                vy(i,0) = value;
	 	vz(i,0) = 0;
		nvx(number_sampling_points-(i+1),0) = value;
        }
	//nvx can also be used as inverted vy since they are equally initialized
        drawSomething(clientID,vx,vy,vz,current_position);
	drawSomething(clientID,nvx,vy,vz,current_position);
	drawSomething(clientID,nvx,nvx,vz,current_position);
	drawSomething(clientID,vx,nvx,vz,current_position);
}

//Draw a circle in the z,y plane 
void drawCircle(int clientID)
{
	Eigen::Matrix<float,5,1> current_position = get_joint_positions(clientID,false);
	Eigen::Matrix<float,5,1> n;
	n<<0,0,0,0,0;
	set_joint_positions(clientID,current_position+n);

	int number_sampling_points = 100;

	Eigen::VectorXf vx (number_sampling_points); // change in z direction
	Eigen::VectorXf vy (number_sampling_points); // change in y direction
	Eigen::VectorXf vz (number_sampling_points); //change in x direction (acctually there is non)

	for (int i = 0; i < number_sampling_points; i++)
	{
		float ifloat = i;
		float value = (1.0/number_sampling_points)*(ifloat+1.0);//Devide the range from 0 to 1 in number_sampling_points pieces
		vz(i,0) = 0.01*std::sin(2*M_PI*value);
		vy(i,0) = 0.01*std::cos(2*M_PI*value);
		vx(i,0) = 0;
		
	}
	drawSomething(clientID,vx,vy,vz,current_position);
}

void drawSpiral(int clientID)
{       
        Eigen::Matrix<float,5,1> current_position = get_joint_positions(clientID,false);
        Eigen::Matrix<float,5,1> n;
        n<<0,0,0,0,0;
        set_joint_positions(clientID,current_position+n);

        int number_sampling_points = 300;

        Eigen::VectorXf vx (number_sampling_points); // change in z direction
        Eigen::VectorXf vy (number_sampling_points); // change in y direction
        Eigen::VectorXf vz (number_sampling_points); //change in x direction (acctually there is non)

        for (int i = 0; i < number_sampling_points; i++)
        {       
                float ifloat = i;
                float value = (0.1/1000)*(ifloat+1.0);//Devide the range from 0 to 1 in 500 pieces
                vz(i,0) = 0.005*std::sin(1000*value);
                vy(i,0) = 0.005*std::cos(1000*value);
                vx(i,0) = value;

        }
        drawSomething(clientID,vx,vy,vz,current_position);
}

void drawHaert(int clientID)
{
        Eigen::Matrix<float,5,1> current_position = get_joint_positions(clientID,false);
        Eigen::Matrix<float,5,1> n;
        n<<0,0,0,0,0;
        set_joint_positions(clientID,current_position+n);

        int number_sampling_points = 50;//Number of sampling points per half cicle
	float r = 0.005; //Radius of the two small cicles

	Eigen::VectorXf vs (number_sampling_points);
	Eigen::VectorXf vc (number_sampling_points);
        Eigen::VectorXf vx (number_sampling_points); // change in x direction
        Eigen::VectorXf vy (number_sampling_points); // change in y direction
        Eigen::VectorXf vz (number_sampling_points); //change in z direction
	Eigen::VectorXf vxn (number_sampling_points); //vx the other way around
	
        for (int i = 0; i < number_sampling_points; i++)
        {
                float ifloat = i;
                float value = (1.0/number_sampling_points)*(ifloat+1.0);//Devide the range from 0 to 1 in number_sampling_points pieces
                vs(i,0) = r*std::sin(M_PI*value);
                vc(i,0) = r*std::cos(M_PI*value);
		vx(i,0) = ((3*r)/number_sampling_points)*(ifloat+1.0);
		vy(i,0) = ((2*r)/number_sampling_points)*(ifloat+1.0);
                vz(i,0) = 0;
		vxn(number_sampling_points-1-i,0) = vx(i,0); 

        }
	//Draw two half cicles
        drawSomething(clientID,vs,vc,vz,current_position);
	drawSomething(clientID,vs,vc,vz,current_position);

	//Draw two other lines
	drawSomething(clientID,vxn,vy,vz,current_position);
	drawSomething(clientID,vx,vy,vz,current_position);
	
}

	
int main(int argc, char* argv[])
{
	//take the time to compare this c++ implemetation with the one using matlab
	
	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

	int portNb = 19999;
	//portNb = atoi(argv[1]);

	int clientID = simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);

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
		//drawHaert(clientID);
		//drawSpiral(clientID);
		drawSquare(clientID,0.02);
		//drawCircle();	

	simxFinish(clientID);
	}
	//std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::chrono::duration<double> elapsed = end- start;	
	std::cout<<"Elapsed time: " <<(end-start).count()<<"s"<<std::endl;
return(0);
}

