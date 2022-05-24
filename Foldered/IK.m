%% inverse Kinematic
x = input("Xee : ");
y = input("Yee : ");
z = input("Zee : ");
alpha = input("alpha : ");
beta = input("beta : ");
gama = input("gama : ");
inverse(x,y,z,alpha,beta,gama)

%% Validation (IK)
t3 = 0;
t2 = 0;
t4 = 0;
t5 = 0;
t6 = 0;
d1 = 0.089159;
d4 = 0.10915;
d5 = 0.09465;
d6 = 0.0823;
a2 = 0.425;
a3 = 0.3922;

for i = -0.5:0.1:0.5
    t = [i*pi, 0, 0, 0, 0, 0];
    [T1, T2, T3, T4, T5, T6, T] = DH(t,d1, d4, d5, d6, a2, a3);
    P = T(1:3,4);
    R = T(1:3,1:3);
    beta = asin(-R(3,1));
    gama = atan2(R(3,2),R(3,3));
    alpha = atan2(R(1,2),R(1,1));
    Q = inverse(P(1),P(2),P(3),alpha,beta,gama);
    l(1) = Link([0, 0.089159,   0,          0,0],'modified');
    l(2) = Link([0, 0,          0,          pi/2,0],'modified');
    l(3) = Link([0, 0,          -0.425,     0,0],'modified');
    l(4) = Link([0, 0.10915,    -0.39225,   0,0],'modified');
    l(5) = Link([0, 0.09465,    0,          pi/2,0],'modified');
    l(6) = Link([0, 0.0823,     0,          -pi/2,0],'modified');
    ur = SerialLink(l);
    ur.plot(Q,'noname')
    pause(0.01)
end

%%
function Q = inverse(x,y,z,alpha,beta,gama)
    d1 = 0.089159;
    d4 = 0.10915;
    d5 = 0.09465;
    d6 = 0.0823	;
    a2 = 0.425;
    a3 = 0.3922;
    P60 = [x;y;z];
    
    %using roll-pitch-yaw Fixed angles
    RZ_alpha = [cos(alpha) -sin(alpha) 0; sin(alpha) cos(alpha) 0; 0 0 1]; 
    RY_beta = [cos(beta) 0 sin(alpha); 0 1 0; -sin(beta) 0 cos(beta)]; 
    RX_gama = [1 0 0; 0 cos(gama) -sin(gama); 0 sin(gama) cos(gama )]; 
    R60 = RZ_alpha * RY_beta * RX_gama;
    T60 = [R60,P60;0 0 0 1];

    %theta1
    P50 = P60 - d6*R60(1:3,3);
    q1(1) = atan2(P50(2),P50(1)) + acos(d4/sqrt(P50(2)^2 + P50(1)^2)) + pi/2;
    q1(2) = atan2(P50(2),P50(1)) - acos(d4/sqrt(P50(2)^2 + P50(1)^2)) + pi/2;

    %theta5
    P61y = (-sin(q1(1)))*P60(1) + (cos(q1(1)))*P60(2); 
    q5(1) = real(acos(-(d4 + P61y)/d6));
    q5(2) = -real(acos(-(d4 + P61y)/d6));

    %theta6
    if (q5(1) <= 10e-5)
        q6 = rand()*2*pi;
    else
        R06 = transpose(R60);
        a = -R06(2,1)*sin(q1(1)) + R06(2,2)*cos(q1(1));
        b = R06(1,1)*sin(q1(1)) - R06(1,2)*cos(q1(1));
        q6 = atan2(a/sin(q5(1)),b/sin(q5(1)));
    end

    [T1, T2, T3, T4, T5, T6, T] = DH([q1(1),0,0,0,q5(1),q6],d1, d4, d5, d6, a2, a3);
    T64 = T5 * T6;
    T41 = transpose(T1) * T60 * transpose(T64);
    P4xz = sqrt(T41(1,4)^2 + T41(3,4)^2);
    
    %theta3
    q3(1) = real(acos(((P4xz)^2 - a2^2 - a3^2)/(2*a2*a3)));
    q3(2) = -real(acos(((P4xz)^2 - a2^2 - a3^2)/(2*a2*a3)));

    %theta2
    phi1 = atan2(-T41(3,4), -T41(1,4)); 
    phi2 = asin(-a3*sin(q3(1))/abs(P4xz));
    q2 = phi1 - phi2;

    [T1, T2, T3, T4, T5, T6, T] = DH([q1(1),q2,q3(1),0,q5(1),q6],d1, d4, d5, d6, a2, a3);
    %theta4
    T64 = T5 * T6;
    T30 = T1*T2*T3;
    T43 = transpose(T30) * T60 * transpose(T64);
    q4 = atan2(T43(2,1), T43(1,1));

    Q = [q1(1),q2,q3(1),q4,q5(1),q6];
end
