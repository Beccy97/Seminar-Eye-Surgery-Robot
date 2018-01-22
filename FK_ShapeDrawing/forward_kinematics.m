function P = forward_kinematics(L)

    %input variables
    L1=L(1);
    L2=L(2);
    L3=L(3);
    L4=L(4);
    L5=L(5);


    %calculate joint positions
    syms q0 q1 q2 q3 q4 q5 
    theta1=atan((L2-L1)/0.017);
    theta2=atan((L3-L4)/0.017);
    q0=(L2+L1)/2;
    q1=theta1;
    q2=(L4+L3)/2;
    q3=theta2;
    q4=L5;
    q5=0;
    l1=0.0330025;
    l2=0;
    l3=0.0400025;
    d1=0.0111654;
    d2=0.029685;
    d3=d1;
    d4=0.0962734;
    s1=sin(q1);
    s3=sin(q3);
    s5=sin(q5);
    c1=cos(q1);
    c3=cos(q3);
    c5=cos(q5);
    
    %calculate needle position
    T6=[-s1*s5-c1*c5*s3, c1*s3*s5-c5*s1, -c1*c3, l2*c1+l1*s1-c1*c3*(d4+q4)-l3*c1*s3;
        c3*c5, -c3*s5, -s3, d2+d3+q2-s3*(d4+q4)+l3*c3;
        c5*s1*s3-c1*s5, -c1*c5-s1*s3*s5, c3*s1, d1+q0+l1*c1-l2*s1+c3*s1*(d4+q4)+l3*s1*s3;
        0, 0, 0, 1]
    
    P(1)=T6(1,4);
    P(2)=T6(2,4);
    P(3)=T6(3,4);
    P(4)=0;
    P(5)=0;
    P(6)=0;
    
end