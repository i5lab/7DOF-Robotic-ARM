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
    T1 = TDH1(d1 ,i*pi);
    T2 = TDH2(t2);
    T3 = TDH3(t3, a2);
    T4 = TDH4(t4, a3, d4);
    T5 = TDH5(t5, d5);
    T6 = TDH6(d6, t6);
    T = (T1 * T2 * T3 * T4 * T5 * T6);
    P = T(1:3,4);
    R = T(1:3,1:3);
    beta = asin(-R(3,1));
    gama = atan2(R(3,2),R(3,3));
    alpha = atan2(R(1,2),R(1,1));
    Q = inverse(P(1),P(2),P(3),alpha,beta,gama);
    l(1) = Link([0, 0.089159,   0,          0,0]);
    l(2) = Link([0, 0,          0,          pi/2,0]);
    l(3) = Link([0, 0,          -0.425,     0,0]);
    l(4) = Link([0, 0.10915,    -0.39225,   0,0]);
    l(5) = Link([0, 0.09465,    0,          pi/2,0]);
    l(6) = Link([0, 0.0823,     0,          -pi/2,0]);
    ur = SerialLink(l);
    ur.plot(Q)
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

    %theta3
    T5 = TDH5(q5(1) , d5);
    T6 = TDH6(d6 , q6);
    T64 = T5 * T6;
    T10 = TDH1(d1, q1(1));
    T41 = transpose(T10) * T60 * transpose(T64);
    P4xz = sqrt(T41(1,4)^2 + T41(3,4)^2);
    q3(1) = real(acos(((P4xz)^2 - a2^2 - a3^2)/(2*a2*a3)));
    q3(2) = -real(acos(((P4xz)^2 - a2^2 - a3^2)/(2*a2*a3)));

    %theta2
    T5 = TDH5(q5(1) , d5);
    T6 = TDH6(d6 , q6(1));
    T64 = T5 * T6;
    T10 = TDH1(d1, q1(1));
    T41 = transpose(T10) * T60 * transpose(T64);
    P4xz = sqrt(T41(1,4)^2 + T41(3,4)^2);
    phi1 = atan2(-T41(3,4), -T41(1,4)); 
    phi2 = asin(-a3*sin(q3(1))/abs(P4xz));
    q2 = phi1 - phi2;

    %theta4
    T5 = TDH5(q5(1) , d5);
    T6 = TDH6(d6 , q6(1));
    T1 = TDH1(d1, q1(1));
    T2 = TDH2(q2(1));
    T3 = TDH3(q3(1), a2);
    T64 = T5 * T6;
    T30 = T1*T2*T3;
    T43 = transpose(T30) * T60 * transpose(T64);
    q4 = atan2(T43(2,1), T43(1,1));

    Q = [q1(1),q2,q3(1),q4,q5(1),q6];
end
