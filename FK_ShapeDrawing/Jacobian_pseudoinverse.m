function L = Jacobian_pseudoinverse(current_position,step)
%Input:current joint positions [L1,L2,L3,L4,L5]
%Input:desired step size (change in position of the end effector [dx;dy;dz;alpha;beta;gamma]
%output:delta([L1,L2,L3,L4,L5]), add this to the current joint positions

% Denavit-Hartenberg-parameters
    L1=0.071;
    L2=0.0335;
    L3=-0.023;
    L4=0.071;
    L5=0.028;
    L6=0.097;

current_position(1)=-current_position(1);
current_position(3)=-current_position(3);

q0=(current_position(2)+current_position(1))/2;
q1=atan((current_position(2)-current_position(1))/0.017);
q2=(current_position(4)+current_position(3))/2;
q3=atan((current_position(4)-current_position(3))/0.017);
q4=current_position(5);

s1=sin(q1);
c1=cos(q1);
s3=sin(q3);
c3=cos(q3);

%as calculated by the MATLAB script
J=[ 0, L2*cos(q1) + L6*cos(q3)*sin(q1) + L5*sin(q1)*sin(q3) + q4*cos(q3)*sin(q1), 0,  cos(q1)*(L6*sin(q3) - L5*cos(q3) + q4*sin(q3)), -cos(q1)*cos(q3),                0;
    0,                                                                         0, 1,          - L6*cos(q3) - L5*sin(q3) - q4*cos(q3),         -sin(q3),                0;
    1, L6*cos(q1)*cos(q3) - L2*sin(q1) + L5*cos(q1)*sin(q3) + q4*cos(q1)*cos(q3), 0, -sin(q1)*(L6*sin(q3) - L5*cos(q3) + q4*sin(q3)),  cos(q3)*sin(q1),                0;
    0,                                                                         0, 0,                                         sin(q1),                0, -cos(q1)*cos(q3);
    0,                                                                         1, 0,                                               0,                0,         -sin(q3);
    0,                                                                         0, 0,                                         cos(q1),                0,  cos(q3)*sin(q1)];
 

Q=J\step;

L(2)=(0.017/2)*tan(Q(2))+Q(1);
L(1)=Q(1)-(0.017/2)*tan(Q(2)); 
L(4)=(0.017/2)*tan(Q(4))+Q(3);
L(3)=Q(3)-(0.017/2)*tan(Q(4)); 
L(5)=Q(5);%flipped coordinate
L(1)=-L(1); %revert  L1,L3
L(3)=-L(3);

end
