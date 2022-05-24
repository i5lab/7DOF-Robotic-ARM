%%
clear all;
close all;
clc;
%DH_modified and Rotations matrices calc.
syms  t1 t2 t3 t4 t6 t5 d1 d4 d5 d6 a2 a3

T1 = TDH1(d1 ,t1);
T2 = TDH2(t2);
T3 = TDH3(t3, a2);
T4 = TDH4(t4, a3, d4);
T5 = TDH5(t5, d5);
T6 = TDH6(d6, t6);
T = (T1 * T2 * T3 * T4 * T5 * T6);

R1 = RDH(T1);
R2 = RDH(T2);
R3 = RDH(T3);
R4 = RDH(T4);
R5 = RDH(T5);
R6 = RDH(T6);
R = (R1 * R2 * R3 * R4 * R5 * R6);

%% Validation (DH)

l(1) = Link([0,0.089159,0,pi/2,0]);
l(2) = Link([0,0,-0.425,0,0]);
l(3) = Link([0,0,-0.39225,0,0]);
l(4) = Link([0,0.10915,0,pi/2,0]);
l(5) = Link([0,0.09465,0,-pi/2,0]);
l(6) = Link([0,0.0823,0,0,0]);
ur = SerialLink(l);
ur.fkine([0,0,0,0,0,0])

%Zero config. for testing
t1 = 0;
t2 = 0;
t3 = 0;
t4 = 0;
t5 = 0;
t6 = 0;
d1 = 0.089159;
d4 = 0.10915;
d5 = 0.09465;
d6 = 0.0823;
a2 = 0.425;
a3 = 0.3922;

T1 = TDH1(d1 ,t1);
T2 = TDH2(t2);
T3 = TDH3(t3, a2);
T4 = TDH4(t4, a3, d4);
T5 = TDH5(t5, d5);
T6 = TDH6(d6, t6);
T = (T1 * T2 * T3 * T4 * T5 * T6)
%%

%%
%inverse Kinematic
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
    l(1) = Link([0,0.089159,0,pi/2,0]);
    l(2) = Link([0,0,-0.425,0,0]);
    l(3) = Link([0,0,-0.39225,0,0]);
    l(4) = Link([0,0.10915,0,pi/2,0]);
    l(5) = Link([0,0.09465,0,-pi/2,0]);
    l(6) = Link([0,0.0823,0,0,0]);
    ur = SerialLink(l);
    ur.plot(Q)
    pause(0.01)
end
%%
%Worksoace
d1 = 0.089159;
d4 = 0.10915;
d5 = 0.09465;
d6 = 0.0823	;
a2 = -0.425;
a3 = -0.3922;
b=1;
for i = 0:0.07:2
    for j = 0:0.07:2
        for k = 0:0.07:2
            for m = 0:0.07:2
                for n = 0:0.07:2
                     for o = 0:0.07:2
                            T1 = TDH1(d1 ,o*pi);
                            T2 = TDH2(n*pi);
                            T3 = TDH3(m*pi, a2);
                            T4 = TDH4(k*pi, a3, d4);
                            T5 = TDH5(j*pi, d5);
                            T6 = TDH6(d6, i*pi);
                            T = T1 * T2 * T3 * T4 * T5 * T6;
                            Position = T(1:3,4);
                            x(b) =Position(1);
                            y(b)= Position(2);
                            z(b) =Position(3);
                            b= b+1;
                     end 
                end 
            end 
        end
    end 
end

P = [transpose(x),transpose(y),transpose(z)];
trisurf(boundary(P),P(:,1),P(:,2),P(:,3));

%% Validation (WorkSpace)
l(1) = Link([0,0.089159,0,pi/2,0]);
l(2) = Link([0,0,-0.425,0,0]);
l(3) = Link([0,0,-0.39225,0,0]);
l(4) = Link([0,0.10915,0,pi/2,0]);
l(5) = Link([0,0.09465,0,-pi/2,0]);
l(6) = Link([0,0.0823,0,0,0]);
ur = SerialLink(l);
ur.plot([0,0,0,0,0,0],'jvec','noname')

%%
%Jacobian_General
disp("General Jacobian : ")
[jv,jw] = Jacobain_General(6,t1,t2,t3,t4,t5,t6);
jv = simplify(jv);
jw = simplify(jw);
J = [jv;jw];
disp(J);
%%
%Jacobian_Velocity_propagation
disp("Velocity propagation Jacobian : ")
[v,w] = velocity_omega(6);
[jv,jw] = Jacobain_Modified(v,w);
jv = simplify(jv);
jw = simplify(jw);
J = [jv;jw];
disp(J);
%%
%singularities
clear singular
b = 1;
l(1) = Link([0,0.089159,0,pi/2,0]);
l(2) = Link([0,0,-0.425,0,0]);
l(3) = Link([0,0,-0.39225,0,0]);
l(4) = Link([0,0.10915,0,pi/2,0]);
l(5) = Link([0,0.09465,0,-pi/2,0]);
l(6) = Link([0,0.0823,0,0,0]);
ur = SerialLink(l);
for i = 0:0.2:1.9
    for j = 0:0.2:1.9
        for k = 0:0.2:1.9
            [jv,jw] = Jacobain_General(6,i*pi,j*pi,k*pi,pi,1.5*pi,0);
            J = [jv;jw];
            J11 = J(1:3,1:3);
            detereminant = det(J11);
            %detereminant = det([jv;jw]);
            if (detereminant < 5e-20)
                 singular(b,1:6) = [i*pi,j*pi,k*pi,pi,1.5*pi,0];
                 %figure(1);
                 %ur.plot([i*pi,j*pi,k*pi,pi,1.5*pi,0]);
                 %pause(0.0001);
                 b = b+1;
            end 
        end
    end 
end
disp(singular);
%%
%Dynamics
[D,C,G] = dynamics_M();

%%
function T1 = TDH1(d1 ,t1)
    T1 = [cos(t1) -sin(t1) 0 0; sin(t1) cos(t1) 0 0; 0 0 1 d1; 0 0 0 1];
end 

function T2 = TDH2(t2)
    T2 = [cos(t2) -sin(t2) 0 0; 0 0 -1 0 ;sin(t2) cos(t2) 0 0; 0 0 0 1];
end

function T3 = TDH3(t3, a2)
    T3 = [cos(t3) -sin(t3) 0 -a2; sin(t3) cos(t3) 0 0; 0 0 1 0; 0 0 0 1];
end

function T4 = TDH4(t4, a3, d4)
    T4 = [cos(t4) -sin(t4) 0 -a3; sin(t4) cos(t4) 0 0; 0 0 1 d4; 0 0 0 1];
end

function T5 = TDH5(t5, d5)
    T5 = [cos(t5) -sin(t5) 0 0; 0 0 -1 -d5 ;sin(t5) cos(t5) 0 0; 0 0 0 1];
end

function T6 = TDH6(d6, t6)
    T6 = [cos(t6) -sin(t6) 0 0; 0 0 1 d6 ;-sin(t6) -cos(t6) 0 0; 0 0 0 1];
end

function Tc1 = TDHc1()
    Tc1 = eye(4);
end 

function Tc2 = TDHc2(a2)
    Tc2 = [1 0 0 -a2/2; 0 1 0 0; 0 0 1 0; 0 0 0 1];
end

function Tc3 = TDHc3(a3, d4)
    Tc3 = [1 0 0 -a3/2; 0 1 0 0; 0 0 1 d4/2; 0 0 0 1];
end

function Tc4 = TDHc4(d5)
    Tc4 = [1 0 0 0; 0 1 0 -d5/2; 0 0 1 0; 0 0 0 1];
end

function Tc5 = TDHc5(d6)
    Tc5 = [1 0 0 0; 0 1 0 d6/2; 0 0 1 0; 0 0 0 1];
end

function Tc6 = TDHc6()
    Tc6 = eye(4);
end 

function Rot = RDH(TDH)
    Rot = TDH(1:3,1:3);
end

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

function [jv,jw] = Jacobain_General(n,t1,t2,t3,t4,t5,t6) 
    %t = sym('t',[1 6]);
    d1 = 0.089159;
    d4 = 0.10915;
    d5 = 0.09465;
    d6 = 0.0823	;
    a2 = -0.425;
    a3 = -0.3922;
    T = cat(3,TDH1(d1,t1),TDH2(t2),TDH3(t3,a2),TDH4(t4,a3,d4),TDH5(t5,d5),TDH6(d6,t6));
    R = cat(3,RDH(TDH1(d1,t1)),RDH(TDH2(t2)),RDH(TDH3(t3,a2)),RDH(TDH4(t4,a3,d4)),RDH(TDH5(t5,d5)),RDH(TDH6(d6,t6)));
    O = T(:,:,1);

    for i = 2:n
        O = O * T(:,:,i);
    end
    
    for i = 1:n
        T_current = eye(4);
        R_current = eye(3);
        for j = 1:i
            T_current = T_current * T(:,:,j);
            R_current = R_current * R(:,:,j);
        end
        jw(1:3,i) = R_current(1:3,3);
        jv(1:3,i) = cross(R_current(1:3,3),((O(1:3,4))-T_current(1:3,4)));
    end
end

function [jv,jw] = Jacobain_Modified(v,w)
    q = sym('q', [1 6]);
    for i=1:6
        xw(i) = diff(w(1),q(i));
        yw(i) = diff(w(2),q(i));
        zw(i) = diff(w(3),q(i));
        xv(i) = diff(v(1),q(i));
        yv(i) = diff(v(2),q(i));
        zv(i) = diff(v(3),q(i));
    end
 
    jw = [xw;yw;zw];
    jv = [xv;yv;zv];
end

function [v,w] = velocity_omega(n)
    w = [0;0;0];
    v = [0;0;0];
    t = sym('t',[1 6]);
    q = sym('q', [1 6]);
    d1 = 0.089159;
    d4 = 0.10915;
    d5 = 0.09465;
    d6 = 0.0823	;
    a2 = -0.425;
    a3 = -0.3922;    T = cat(3,TDH1(d1,t(1)),TDH2(t(2)),TDH3(t(3),a2),TDH4(t(4),a3,d4),TDH5(t(5),d5),TDH6(d6,t(6)));
    R = cat(3,RDH(TDH1(d1,t(1))),RDH(TDH2(t(2))),RDH(TDH3(t(3),a2)),RDH(TDH4(t(4),a3,d4)),RDH(TDH5(t(5),d5)),RDH(TDH6(d6,t(6))));
    
    for i = 1:n
        P = T(:,:,i);
        v = transpose(R(:,:,i)) * (v + cross(w,P(1:3,4)));
        w = transpose(R(:,:,i)) * w + [0;0;q(i)];
    end
    
    w = RDH(TDH1(d1,t(1)))*RDH(TDH2(t(2)))*RDH(TDH3(t(3),a2))*RDH(TDH4(t(4),a3,d4))*RDH(TDH5(t(5),d5))*RDH(TDH6(d6,t(6)))*w;
    v = RDH(TDH1(d1,t(1)))*RDH(TDH2(t(2)))*RDH(TDH3(t(3),a2))*RDH(TDH4(t(4),a3,d4))*RDH(TDH5(t(5),d5))*RDH(TDH6(d6,t(6)))*v;
end

function [D,C,G] = dynamics_M ()
    t = sym('t', [1 6]);
    m = sym('m', [1 6]);
    d1 = 0.089159;
    d4 = 0.10915;
    d5 = 0.09465;
    d6 = 0.0823	;
    a2 = -0.425;
    a3 = -0.3922;
    IXX = sym('IXX', [1 6]);
    IYY = sym('IYY', [1 6]);
    IZZ = sym('IZZ', [1 6]);
    IXY = sym('IXY', [1 6]);
    IXZ = sym('IXZ', [1 6]);
    IYZ = sym('IYZ', [1 6]);
    for i = 1:6 
        I(:,:,i) = [IXX(i) IXY(i) IXZ(i);IXY(i) IYY(i) IYZ(i) ;IXZ(i) IYZ(i) IZZ(i)];
        mass(i) = m(i);
    end

    D = sym(zeros(6,6));
    C = sym(zeros(6,6,6));
    rc = sym(zeros(1,6));
    G = sym(zeros(6,1));
    P=0;
    
    jw = sym(zeros(3,6));
    jv = sym(zeros(3,6));
   
    T = cat(3,TDH1(d1,t(1)),TDH2(t(2)),TDH3(t(3),a2),TDH4(t(4),a3,d4),TDH5(t(5),d5),TDH6(d6,t(6)));
    Tc = cat(3,TDHc1(),TDHc2(a2),TDHc3(a3, d4),TDHc4(d5),TDHc5(d6),TDHc6());
    R = cat(3,RDH(TDH1(d1,t(1))),RDH(TDH2(t(2))),RDH(TDH3(t(3),a2)),RDH(TDH4(t(4),a3,d4)),RDH(TDH5(t(5),d5)),RDH(TDH6(d6,t(6))));
    
    T_current = eye(4);
    R_current = eye(3);
    for i=1:6
        T_current = T_current * T(:,:,i);
        Tc_current = T_current * Tc(:,:,i);
        Tc_stack(:,:,i) = Tc_current;
        R_current = R_current * R(:,:,i);
        R_stack(:,:,i) = R_current;
        jw(:,i) = R_current(:,3);
        for n = 1:i
            jv(1:3,n) = cross(R_stack(:,3,n),(Tc_stack(1:3,4,i)-T(1:3,4,n)));
        end
        rc(i) = -jv(1,1);
        D = D + (mass(i)*transpose(jv)*jv) + (transpose(jw)*R_stack(:,:,i)*I(:,:,i)*transpose(R_stack(:,:,i))*jw);
    end    
    
    for k = 1:6
        for i = 1:6
            for j = i:6
                C(i,j,k) = 0.5*(diff(D(k,j),t(i)) + diff(D(k,i),t(j)) - diff(D(i,j),t(k)));
                P = P + (9.8*rc(i)*mass(i));
            end 
        end 
    end
    
    for i= 1:6
        G(i) = diff(P,t(i));
    end   
    
%     D = simplify(D);
%     C = simplify(C);
    G = simplify(G);
end 