%% FK
clc
clear
close
t = sym('t', [1 6]);
syms d1 d4 d5 d6 a2 a3;
[T1, T2, T3, T4, T5, T6, T] = DH(t,d1, d4, d5, d6, a2, a3);

%% Validation (FK)

%Zero config. for testing
t =zeros(1,6);
d1 = 0.089159;
d4 = 0.10915;
d5 = 0.09465;
d6 = 0.0823;
a2 = 0.425;
a3 = 0.3922;
[T1, T2, T3, T4, T5, T6, T] = DH(t,d1, d4, d5, d6, a2, a3)

l(1) = Link([0, 0.089159,   0,          0,0],'modified');
l(2) = Link([0, 0,          0,          pi/2,0],'modified');
l(3) = Link([0, 0,          -0.425,     0,0],'modified');
l(4) = Link([0, 0.10915,    -0.39225,   0,0],'modified');
l(5) = Link([0, 0.09465,    0,          pi/2,0],'modified');
l(6) = Link([0, 0.0823,     0,          -pi/2,0],'modified');
ur = SerialLink(l);
peter_corke = ur.fkine([0,0,0,0,0,0])