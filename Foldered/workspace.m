%% Worksoace
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