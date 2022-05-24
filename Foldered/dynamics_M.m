
%% Dynamics
[D,C,G] = dynamics_M();

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
   
    [T1, T2, T3, T4, T5, T6] = DH(t,d1, d4, d5, d6, a2, a3);
    T = cat(3,T1, T2, T3, T4, T5, T6);
    [R1, R2, R3, R4, R5, R6] = RDH(T1, T2, T3, T4, T5, T6, T);
    R = cat(3,R1, R2, R3, R4, R5, R6);
    [TC1, TC2, TC3, TC4, TC5, TC6] = DHC(d4, d5, d6, a2, a3);
    Tc = cat(3,TC1, TC2, TC3, TC4, TC5, TC6);
    
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