%% Worksoace
d1 = 0.089159;
d4 = 0.10915;
d5 = 0.09465;
d6 = 0.0823	;
a2 = -0.425;
a3 = -0.3922;
b = 1;
for i = 0:0.07:2
    for j = 0:0.07:2
        for k = 0:0.07:2
            for m = 0:0.07:2
                for n = 0:0.07:2
                     for o = 0:0.07:2
                         t = [i*pi, j*pi, k*pi, m*pi, n*pi, o*pi];
                         [T1, T2, T3, T4, T5, T6, T] = DH(t,d1, d4, d5, d6, a2, a3);
                         Position = T(1:3,4);
                         x(b) =Position(1);
                         y(b)= Position(2);
                         z(b) =Position(3);
                         b = b+1;
                     end 
                end 
            end 
        end
    end 
end

P = [transpose(x),transpose(y),transpose(z)];
trisurf(boundary(P),P(:,1),P(:,2),P(:,3));

%% Validation (WorkSpace)
l(1) = Link([0, 0.089159,   0,          0,0],'modified');
l(2) = Link([0, 0,          0,          pi/2,0],'modified');
l(3) = Link([0, 0,          -0.425,     0,0],'modified');
l(4) = Link([0, 0.10915,    -0.39225,   0,0],'modified');
l(5) = Link([0, 0.09465,    0,          pi/2,0],'modified');
l(6) = Link([0, 0.0823,     0,          -pi/2,0],'modified');
ur = SerialLink(l);

for i = 0:0.07:2
    for j = 0:0.07:2
        for k = 0:0.07:2
            for m = 0:0.07:2
                for n = 0:0.07:2
                     for o = 0:0.07:2
                         t = [i*pi, j*pi, k*pi, m*pi, n*pi, o*pi];
                         ur.plot(t,'jvec','noname')
                         pause(0.001)
                     end 
                end 
            end 
        end
    end 
end
