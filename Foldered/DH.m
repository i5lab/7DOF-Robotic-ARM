function [T1, T2, T3, T4, T5, T6, T] = DH(t,d1, d4, d5, d6, a2, a3)
    T1 = [cos(t(1)) -sin(t(1)) 0 0; sin(t(1)) cos(t(1)) 0 0; 0 0 1 d1; 0 0 0 1];
    T2 = [cos(t(2)) -sin(t(2)) 0 0; 0 0 -1 0 ;sin(t(2)) cos(t(2)) 0 0; 0 0 0 1];
    T3 = [cos(t(3)) -sin(t(3)) 0 -a2; sin(t(3)) cos(t(3)) 0 0; 0 0 1 0; 0 0 0 1];
    T4 = [cos(t(4)) -sin(t(4)) 0 -a3; sin(t(4)) cos(t(4)) 0 0; 0 0 1 d4; 0 0 0 1];
    T5 =[cos(t(5)) -sin(t(5)) 0 0; 0 0 -1 -d5 ;sin(t(5)) cos(t(5)) 0 0; 0 0 0 1];
    T6 = [cos(t(6)) -sin(t(6)) 0 0; 0 0 1 d6 ;-sin(t(6)) -cos(t(6)) 0 0; 0 0 0 1];
    T = T1*T2*T3*T4*T5*T6;
end