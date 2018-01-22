function position = get_joint_positions(vrep,clientID,first_call)

    % Handles
    [~,L1]=vrep.simxGetObjectHandle(clientID,'Needle_L1',vrep.simx_opmode_blocking);
    [~,L2]=vrep.simxGetObjectHandle(clientID,'Needle_L2',vrep.simx_opmode_blocking);
    [~,L3]=vrep.simxGetObjectHandle(clientID,'Needle_L3',vrep.simx_opmode_blocking);
    [~,L4]=vrep.simxGetObjectHandle(clientID,'Needle_L4',vrep.simx_opmode_blocking);
    [~,L5]=vrep.simxGetObjectHandle(clientID,'Needle_L5',vrep.simx_opmode_blocking);


    if first_call==1
        [~,position(1)]=vrep.simxGetJointPosition(clientID,L1,vrep.simx_opmode_streaming);%use buffer on subsequent calls
        [~,position(2)]=vrep.simxGetJointPosition(clientID,L2,vrep.simx_opmode_streaming);
        [~,position(3)]=vrep.simxGetJointPosition(clientID,L3,vrep.simx_opmode_streaming);
        [~,position(4)]=vrep.simxGetJointPosition(clientID,L4,vrep.simx_opmode_streaming);
        [~,position(5)]=vrep.simxGetJointPosition(clientID,L5,vrep.simx_opmode_streaming);
    end
    
    [~,position(1)]=vrep.simxGetJointPosition(clientID,L1,vrep.simx_opmode_buffer);%use buffer on subsequent calls
    [~,position(2)]=vrep.simxGetJointPosition(clientID,L2,vrep.simx_opmode_buffer);
    [~,position(3)]=vrep.simxGetJointPosition(clientID,L3,vrep.simx_opmode_buffer);
    [~,position(4)]=vrep.simxGetJointPosition(clientID,L4,vrep.simx_opmode_buffer);
    [~,position(5)]=vrep.simxGetJointPosition(clientID,L5,vrep.simx_opmode_buffer);
end