function [A,R,R_t,P,P_com,flag] = ArmMatrix(inputFile)

fid = fopen(inputFile,'r')
NJ = fscanf(fid,'%f',1);
DOF = fscanf(fid,'%f',1);

%the input DH table is in distal and needs to be comverted into proximal
alpha = zeros(NJ+1,1);
a = zeros(NJ+1,1);
theta = zeros(NJ+1,1);
d = zeros(NJ+1,1);
flag = zeros(NJ,1);

for i = 1:NJ
    alpha(i+1) = fscanf(fid,'%f',1)*(pi/180);
    a(i+1) = fscanf(fid,'%f',1);
    d(i) = fscanf(fid,'%f',1);
    theta(i) = fscanf(fid,'%f',1)*(pi/180);
    flag(i) = fscanf(fid,'%f',1);
end

R = zeros(3,3,NJ+1);
R_t = zeros(3,3,NJ+1);
P = zeros(3,1,NJ+1);
P_com = zeros(3,1,NJ);
for i = 0:NJ 
    %transformation matrix from i frame to i+1 frame
    %for n links in proximal, there will be n+1 homogeneous transformations
    %for n links in proximal, there will be n+1 rotation matrices
    %for n links in proximal, there will be n+1 position vectors
    A{i+1} =     [cos(theta(i+1)),-sin(theta(i+1)),0,a(i+1); 
                  sin(theta(i+1))*cos(alpha(i+1)), cos(theta(i+1))*cos(alpha(i+1)), -sin(alpha(i+1)), -sin(alpha(i+1))*d(i+1);
                  sin(theta(i+1))*sin(alpha(i+1)), cos(theta(i+1))*sin(alpha(i+1)), cos(alpha(i+1)), cos(alpha(i+1))*d(i+1);
                  0                              ,    0                           ,    0           ,   1];         
    %rotation matrix from i frame to i+1 frame
    R(:,:,i+1) = A{i+1}(1:3,1:3);
    %rotation matrix from i+1 frame to i frame
    %inverse of rotation matrix from frame i to frame i+1
    %rotation matrix is orthogonal, we just need to find its transpose
    R_t(:,:,i+1) = R(:,:,i+1)';
    
    %position vector of i+1 frame with respect to i frame
    P(:,:,i+1) = A{i+1}(1:3,4);
    %position vector of link i COM with respect to i frame (proximal frame)
    %n such vectors for n links
    if i~= 0
        if flag(i) == 1
            P_com(:,:,i) = [a(i+1)/2;0;0];
        else
            P_com(:,:,i) = [0;0;-d(i)/2];
        end
       
    end
end

end


   