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

%     current_position=get_joint_positions(vrep,clientID,0); % get the robot position from vrep
%     set_joint_positions(vrep,clientID,current_position+[0,0,0,0,0]); % set the position in matlab with same coords from vrep
%     
% %     for i=0:10
% %         set_joint_positions(vrep,clientID,get_joint_positions(vrep,clientID,0)+[0,0,0.001,0.001,0])
% %         pause(1);
% %     end
%     
%     t=linspace(0,0.10,1000); % divide range from 0 to 0.1 into 1000 equivalent parts (x - direction)
%    v=0.005*sin(1000*t); % z velocity
%    v0 = t * 0; % x velocity
%    v2=0.005*cos(1000*t); % y velocity
%    
%     for i=2:500
%       
%         current_position=get_joint_positions(vrep,clientID,0);
%         calculation_position=current_position+[+0.0152,-0.0103,-0.0038,+0.0038,0];
%         delta_L=Jacobian_pseudoinverse(calculation_position,[t(i)-t(i-1);v2(i)-v2(i-1);v(i)-v(i-1);0;0;0]) % calculate delta
%         set_joint_positions(vrep,clientID,current_position+delta_L); % set new point
%     end

    % draw square
    
%      current_position=get_joint_positions(vrep,clientID,0); % get the robot position from vrep
%     set_joint_positions(vrep,clientID,current_position+[0,0,0,0,0]); % set the position in matlab with same coords from vrep
%     
%     sampling_points_per_edge = 50; %The number of sampling points per edge of the square
%     length = 0.02; %The actual length of one edge of the square is squr(2)*length (see Pythagoras) 
%     t=linspace(0,length,sampling_points_per_edge); % divide range from 0 to length into smapling_points_per_edge equivalent parts (x - direction)
%    v0 = t * 0; % initialize a vector of null values having size of samling_points_per_edge
%    
%    % Draw diagonal between x and y axis with pos x and pos y values
%     for i=2:sampling_points_per_edge
%       
%         current_position=get_joint_positions(vrep,clientID,0);
%         %Second val was -0.0103
%         calculation_position=current_position+[+0.0152,-0.0152,-0.0038,+0.0038,0];
%         delta_L=Jacobian_pseudoinverse(calculation_position,[t(i)-t(i-1);t(i)-t(i-1);v0(i)-v0(i-1);0;0;0]) % calculate delta
%         set_joint_positions(vrep,clientID,current_position+delta_L); % set new point
%     end
%    
%     % Draw diagonal between x and y axis with neg x and pos y values
%     for i=2:sampling_points_per_edge
%         current_position=get_joint_positions(vrep,clientID,0);
%         calculation_position=current_position+[+0.0152,-0.0152,-0.0038,+0.0038,0];
%         delta_L=Jacobian_pseudoinverse(calculation_position,[-(t(i)-t(i-1));t(i)-t(i-1);v0(i)-v0(i-1);0;0;0]) % calculate delta
%         set_joint_positions(vrep,clientID,current_position+delta_L); % set new point
%     end
%     
%     % Draw diagonal between x and y axis with neg x and neg y values
%     for i=2:sampling_points_per_edge
%         current_position=get_joint_positions(vrep,clientID,0);
%         calculation_position=current_position+[+0.0152,-0.0152,-0.0038,+0.0038,0];
%         delta_L=Jacobian_pseudoinverse(calculation_position,[-(t(i)-t(i-1));(-1)*(t(i)-t(i-1));v0(i)-v0(i-1);0;0;0]) % calculate delta
%         set_joint_positions(vrep,clientID,current_position+delta_L); % set new point
%     end
%     
%     % Draw diagonal between x and y axis with pos x and neg y values
%     for i=2:sampling_points_per_edge
%       
%         current_position=get_joint_positions(vrep,clientID,0);
%         calculation_position=current_position+[+0.0152,-0.0152,-0.0038,+0.0038,0];
%         delta_L=Jacobian_pseudoinverse(calculation_position,[t(i)-t(i-1);-(t(i)-t(i-1));v0(i)-v0(i-1);0;0;0]) % calculate delta
%         set_joint_positions(vrep,clientID,current_position+delta_L); % set new point
%     end
    
    
    % draw heart
    
    current_position=get_joint_positions(vrep,clientID,0); % get the robot position from vrep
    set_joint_positions(vrep,clientID,current_position+[0,0,0,0,0]); % set the position in matlab with same coords from vrep
    
   r = 0.005 ; %Radius of the two small circles
   sampling_points_per_half_circle = 50; %Number of Sampling points used for each of the first 2 Half Circles
   
   t=linspace(0,1,sampling_points_per_half_circle); % divide range from 0 to 1 into  sampling_points_per_half_circle equivalent parts 
   vs=r*sin(pi*t); % Vector of lentgth sampling_points_per_half_circle sampling the first half of sin 
   v0 = t * 0; % 0 Vector of length sampling_points_per_half_circle 
   vc=r*cos(pi*t); % Vector of length sampling_points_per_half_circle sampling the first half of cos
   
   %Draw two half circles of rad r laying in the x,y plane dimension
    for i=2:sampling_points_per_half_circle 
      
        current_position=get_joint_positions(vrep,clientID,0);
        calculation_position=current_position+[+0.0152,-0.0152,-0.0038,+0.0038,0];
        delta_L=Jacobian_pseudoinverse(calculation_position,[vs(i)-vs(i-1);vc(i)-vc(i-1);v0(i)-v0(i-1);0;0;0]) % calculate delta
        set_joint_positions(vrep,clientID,current_position+delta_L); % set new point
    end
    
    for i=2:sampling_points_per_half_circle
      
        current_position=get_joint_positions(vrep,clientID,0);
        calculation_position=current_position+[+0.0152,-0.0152,-0.0038,+0.0038,0];
        delta_L=Jacobian_pseudoinverse(calculation_position,[vs(i)-vs(i-1);vc(i)-vc(i-1);v0(i)-v0(i-1);0;0;0]) % calculate delta
        set_joint_positions(vrep,clientID,current_position+delta_L); % set new point
    end
    
    % herz unterseite beginn
    % Achtung von den Koordinaten her steht das Herz auf dem Kopf !!
    
    number_of_sampling_points_per_line = 50; 
    vx = linspace(0,3*r,number_of_sampling_points_per_line); %Die Spitze des Herzes sollte die Koordinaten x = -3r und y = -r haben 
    vy = linspace(0,2*r,number_of_sampling_points_per_line); %(Sonst siehts komisch aus)
    
    % Draw Line up to the top
    for i=2:number_of_sampling_points_per_line
      
        current_position=get_joint_positions(vrep,clientID,0);
        calculation_position=current_position+[+0.0152,-0.0152,-0.0038,+0.0038,0];
        delta_L=Jacobian_pseudoinverse(calculation_position,[-(vx(i)-vx(i-1));vy(i)-vy(i-1);v0(i)-v0(i-1);0;0;0]) % calculate delta
        set_joint_positions(vrep,clientID,current_position+delta_L); % set new point
    end
    
     %Draw the "Back Line"
     for i=2:number_of_sampling_points_per_line
      
        current_position=get_joint_positions(vrep,clientID,0);
        calculation_position=current_position+[+0.0152,-0.0152,-0.0038,+0.0038,0];
        delta_L=Jacobian_pseudoinverse(calculation_position,[vx(i)-vx(i-1);vy(i)-vy(i-1);v0(i)-v0(i-1);0;0;0]) % calculate delta
        set_joint_positions(vrep,clientID,current_position+delta_L); % set new point
    end
    
   
%       move to the target position in a straight line
     
%     step_width=0.001; %take 1mm-steps
%     target_vector=target_position-needle_position;
% 
%     while (norm(target_vector) > step_width)
%         current_position=get_joint_positions(vrep,clientID,0);
%         step=target_vector/(norm(target_vector)*(1/step_width))
%         calculation_position=current_position+[+0.0152,-0.0103,-0.0038,+0.0038,0];
%         delta_L=Jacobian_pseudoinverse(calculation_position,[step(1);step(2);step(3);0;0;0])
%         set_joint_positions(vrep,clientID,current_position+delta_L);
%         [~,needle_position]=vrep.simxGetObjectPosition(clientID,needle_tip,calculation_zero,vrep.simx_opmode_buffer);
%         [~,target_position]=vrep.simxGetObjectPosition(clientID,target,calculation_zero,vrep.simx_opmode_blocking);
%         target_vector=target_position-needle_position;
%     end

    vrep.simxFinish(-1);
end

vrep.delete();
