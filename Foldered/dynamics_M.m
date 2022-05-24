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