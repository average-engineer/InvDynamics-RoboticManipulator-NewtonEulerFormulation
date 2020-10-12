function [J] = inertia_matrix_COM(NJ,m,l,flag)

for i = 1:NJ
    if flag(i) == 1
        Ixx(i) = 0;
        Iyy(i) = (1/12)*m(i)*(l(i))^2;
        Izz(i) = Iyy(i);
        Ixz(i) = 0;
        Iyz(i) = 0;
        Ixy(i) = 0;
    else 
        Izz(i) = 0;
        Iyy(i) = (1/12)*m(i)*(l(i))^2;
        Ixx(i) = Iyy(i);
        Ixz(i) = 0;
        Iyz(i) = 0;
        Ixy(i) = 0;
    end

end

for i = 1:NJ
    J{i} = [Ixx(i),-Ixy(i),-Ixz(i);-Ixy(i),Iyy(i),-Iyz(i);-Ixz(i),-Iyz(i),Izz(i)];
end

end