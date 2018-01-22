function S = set_joint_positions(vrep,clientID,position)

    % Handles
    [~,L1]=vrep.simxGetObjectHandle(clientID,'Needle_L1',vrep.simx_opmode_blocking);
    [~,L2]=vrep.simxGetObjectHandle(clientID,'Needle_L2',vrep.simx_opmode_blocking);
    [~,L3]=vrep.simxGetObjectHandle(clientID,'Needle_L3',vrep.simx_opmode_blocking);
    [~,L4]=vrep.simxGetObjectHandle(clientID,'Needle_L4',vrep.simx_opmode_blocking);
    [~,L5]=vrep.simxGetObjectHandle(clientID,'Needle_L5',vrep.simx_opmode_blocking);

    
    vrep.simxPauseCommunication(19999,1);

    vrep.simxSetJointTargetPosition(clientID,L1,position(1),vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetPosition(clientID,L2,position(2),vrep.simx_opmode_oneshot);   
    vrep.simxSetJointTargetPosition(clientID,L3,position(3),vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetPosition(clientID,L4,position(4),vrep.simx_opmode_oneshot);  
    vrep.simxSetJointTargetPosition(clientID,L5,position(5),vrep.simx_opmode_oneshot);  

    vrep.simxPauseCommunication(19999,0);
end

