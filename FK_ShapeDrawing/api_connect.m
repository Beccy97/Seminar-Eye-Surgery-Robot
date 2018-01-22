clear
clc

vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp ('Connected')
        
    %Handles
    [~,needle_tip]=vrep.simxGetObjectHandle(clientID,'Needle_tip',vrep.simx_opmode_blocking);
    [~,target]=vrep.simxGetObjectHandle(clientID,'IK_target',vrep.simx_opmode_blocking);
    [~,base]=vrep.simxGetObjectHandle(clientID,'Needle_base',vrep.simx_opmode_blocking);

    
    % positions
    current_position=get_joint_positions(vrep,clientID,1);
    [~,needle_position]=vrep.simxGetObjectPosition(clientID,needle_tip,base,vrep.simx_opmode_streaming);
    pause(2)
    
    [~,needle_position]=vrep.simxGetObjectPosition(clientID,needle_tip,base,vrep.simx_opmode_buffer);
    [~,angle_error]=vrep.simxGetObjectOrientation(clientID,target,needle_tip,vrep.simx_opmode_buffer);
    [~,target_position]=vrep.simxGetObjectPosition(clientID,target,base,vrep.simx_opmode_blocking);

    %draw a spiral

    current_position=get_joint_positions(vrep,clientID,0);
    set_joint_positions(vrep,clientID,current_position+[0,0,0,0,0]);
    
%     for i=0:10
%         set_joint_positions(vrep,clientID,get_joint_positions(vrep,clientID,0)+[0,0,0.001,0.001,0])
%         pause(1);
%     end
    
    t=linspace(0,0.10,1000);
    v=0.005*sin(1000*t);
    v2=0.005*cos(1000*t);
   
    for i=2:500
      
        current_position=get_joint_positions(vrep,clientID,0);
        calculation_position=current_position+[+0.0152,-0.0103,-0.0038,+0.0038,0];
        delta_L=Jacobian_pseudoinverse(calculation_position,[t(i)-t(i-1);v2(i)-v2(i-1);v(i)-v(i-1);0;0;0])
        set_joint_positions(vrep,clientID,current_position+delta_L);
   end
    
    
%       move to the target position in a straight line
     
    step_width=0.001; %take 1mm-steps
    target_vector=target_position-needle_position;

    while (norm(target_vector) > step_width)
        current_position=get_joint_positions(vrep,clientID,0);
        step=target_vector/(norm(target_vector)*(1/step_width))
        calculation_position=current_position+[+0.0152,-0.0103,-0.0038,+0.0038,0];
        delta_L=Jacobian_pseudoinverse(calculation_position,[step(1);step(2);step(3);0;0;0])
        set_joint_positions(vrep,clientID,current_position+delta_L);
        [~,needle_position]=vrep.simxGetObjectPosition(clientID,needle_tip,calculation_zero,vrep.simx_opmode_buffer);
        [~,target_position]=vrep.simxGetObjectPosition(clientID,target,calculation_zero,vrep.simx_opmode_blocking);
        target_vector=target_position-needle_position;
    end

    vrep.simxFinish(-1);
end

vrep.delete();
