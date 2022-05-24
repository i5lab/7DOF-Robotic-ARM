%% Jacobian_General
disp("General Jacobian : ")
t = sym('t', [1 6]);
[jv,jw] = Jacobain_General(t);
jv = simplify(jv);
jw = simplify(jw);
J = vpa([jv;jw]);
disp(J);
%% Jacobian_Velocity_propagation
disp("Velocity propagation Jacobian : ")
[v,w] = velocity_omega();
[jv,jw] = Jacobain_Velocity_propagation(v,w);
jv = simplify(jv);
jw = simplify(jw);
J = vpa([jv;jw]);
disp(J);
%% singularities
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
function [jv,jw] = Jacobain_General(t) 
%     syms d1 d4 d5 d6 a2 a3
    d1 = 0.089159;
    d4 = 0.10915;
    d5 = 0.09465;
    d6 = 0.0823;
    a2 = -0.425;
    a3 = -0.3922;
    [T1, T2, T3, T4, T5, T6] = DH(t,d1, d4, d5, d6, a2, a3);
    T = cat(3,T1, T2, T3, T4, T5, T6);
    [R1, R2, R3, R4, R5, R6] = RDH(T1, T2, T3, T4, T5, T6, T);
    R = cat(3,R1, R2, R3, R4, R5, R6);
    O = T(:,:,1);

    for i = 2:6
        O = O * T(:,:,i);
    end
    
    for i = 1:6
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

function [v,w] = velocity_omega()
    w = [0;0;0];
    v = [0;0;0];
    t = sym('t',[1 6]);
    q = sym('q', [1 6]);
    d1 = 0.089159;
    d4 = 0.10915;
    d5 = 0.09465;
    d6 = 0.0823;
    a2 = -0.425;
    a3 = -0.3922;
%     syms d1 d4 d5 d6 a2 a3
    [T1, T2, T3, T4, T5, T6] = DH(t,d1, d4, d5, d6, a2, a3);
    T = cat(3,T1, T2, T3, T4, T5, T6);
    [R1, R2, R3, R4, R5, R6, RT] = RDH(T1, T2, T3, T4, T5, T6, T);
    R = cat(3,R1, R2, R3, R4, R5, R6);
    
    for i = 1:6
        P = T(:,:,i);
        v = transpose(R(:,:,i)) * (v + cross(w,P(1:3,4)));
        w = transpose(R(:,:,i)) * w + [0;0;q(i)];
    end
    
    w = RT*w;
    v = RT*v;
end

function [jv,jw] = Jacobain_Velocity_propagation(v,w)
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
