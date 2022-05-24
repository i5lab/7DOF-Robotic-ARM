%% Representaton of orientatons
%Eigenvector and theta
disp("Eigenvector and theta : ")
theta = acos((T(1,1) + T(2,2) + T(3,3) - 1)/2);
theta = simplify(theta)
k = 1/sin(theta) * [T(3,2)-T(2,3);T(1,3)-T(3,1);T(2,1)-T(1,2)];
k = simplify(k)

%Fixed angles
disp("Fixed angles : ")
beta = atan2(-T(3,1),sqrt(T(1,1)^2 + T(2,1)^2));
alpha = atan2(T(2,1)/cos(beta) , T(1,1)/cos(beta));
gama = atan2(T(3,2)/cos(beta) , T(3,3)/cos(beta));
beta = simplify(beta)
alpha = simplify(alpha)
gama = simplify(gama)

%Euler angles
disp("Euler angles : ")
beta = atan2(sqrt(T(3,1)^2 + T(3,2)^2), T(3,3));
alpha = atan2(T(2,3)/sin(beta) , T(1,3)/sin(beta));
gama = atan2(T(3,2)/sin(beta) , -T(3,1)/sin(beta));
beta = simplify(beta)
alpha = simplify(alpha)
gama = simplify(gama)

%Quaternion
disp("Quaternion : ")
q0 = cos(theta/2);
q1 = k(1) * sin(theta/2);
q2 = k(2) * sin(theta/2);
q3 = k(3) * sin(theta/2);
q0 = simplify(q0)
q1 = simplify(q1)
q2 = simplify(q2)
q3 = simplify(q3)