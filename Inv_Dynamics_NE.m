%% CODE BASED ON NEWTON-EULER DYNAMIC FORMULATION OF A MANIPULATOR
%NJ = No. of joints
%NF = No. of frames
%flag = type of joint (0 for prismatic/1 for rotary)
%%
clear all 
close all
clc
%% Input file for DH parameters
inputFile = 'dhParameters_2link.txt';
%number of joints
NJ = 2;
%mass of links
link_masses = zeros(NJ,1);
link_masses(1) = 1;
link_masses(2) = 0.5;
% link_masses(3) = 0.5;

%Link lengths
link_lengths = ones(NJ,1);

%joint speeds (in joint space)
joint_velocity = zeros(NJ,1);
joint_velocity(1) = 0.5;%radians per second
joint_velocity(2) = 0.25; %radians per second
% joint_velocity(3) = 0.125;%meters per second
%joint velocity vectors
for i  = 1:NJ
    joint_velocity_vector{i} = [0;0;joint_velocity(i)];
end

%joint accelerations (in joint space)
joint_accelerations = ones(NJ,1);%radians per second
for i = 1:NJ
    joint_acceleration_vector{i} = [0;0;joint_accelerations(i)];
end

%tranformation matrix from i frame to i+1 frame 
%also outputing the rotation matrix from i to i+1 and vice versa
%position matrix from i to i+1 and from i to COM of link i
%the type of joint (flag) is also output
[A,R,R_t,P,P_com,flag] = ArmMatrix(inputFile);

%inertia matrix for each link wrt to its COM
J = inertia_matrix_COM(NJ,link_masses,link_lengths,flag)

%outward iterations
%outputs
%angular velocity of each link (in cartesian space)
%angular acceleration of each link
%linear acceleration of each link (in cartesian space)
%linear acceleration of each link's COM (in cartesian space)

ang_vel = zeros(3,1,NJ+1);
ang_acc = zeros(3,1,NJ+1);
linear_acc = zeros(3,1,NJ+1);
linear_acc(2,1,1) = 9.81;%to include gravity, the base frame is given an acceleration of g upwards
linear_acc_com = zeros(3,1,NJ);
inert_force = zeros(3,1,NJ);
inert_torque = zeros(3,1,NJ);

for i = 1:NJ
    %if the joint is revolute
    if flag(i) == 1
        ang_vel(:,:,i+1) = R_t(:,:,i)*ang_vel(:,:,i) + joint_velocity_vector{i};
        ang_acc(:,:,i+1) = R_t(:,:,i)*ang_acc(:,:,i) + cross(R_t(:,:,i)*ang_vel(:,:,i),joint_velocity_vector{i}) + joint_acceleration_vector{i};
        linear_acc(:,:,i+1) = R_t(:,:,i)*(cross(ang_acc(:,:,i),P(:,:,i)) + cross(ang_vel(:,:,i),cross(ang_vel(:,:,i),P(:,:,i))) + linear_acc(:,:,i));
        
% if the joint is prismatic
    else
        ang_vel(:,:,i+1) = R_t(:,:,i)*ang_vel(:,:,i);
        ang_acc(:,:,i+1) = R_t(:,:,i)*ang_vel(:,:,i);
        linear_acc(:,:,i+1) = R_t(:,:,i)*(cross(ang_acc(:,:,i),P(:,:,i)) + cross(ang_vel(:,:,i),cross(ang_vel(:,:,i),P(:,:,i))) + linear_acc(:,:,i) + 2*cross(ang_vel(:,:,i+1),joint_velocity_vector{i}) + joint_acceleration_vector{i});
    
    end
    linear_acc_com(:,:,i) = cross(ang_acc(:,:,i+1),P_com(:,:,i)) + cross(ang_vel(:,:,i+1),cross(ang_vel(:,:,i+1),P_com(:,:,i))) + linear_acc(:,:,i+1);
    
    %inertial force
    inert_force(:,:,i) = link_masses(i)*linear_acc_com(:,:,i);
    inert_torque(:,:,i) = J{i}*ang_acc(:,:,i+1) + cross(ang_vel(:,:,i+1), (J{i}*ang_vel(:,:,i+1)));
end


%inward iterations
%the link forces and torques will be output
%joint torques/forces can then be found out from the link forces/torques
force = zeros(3,1,NJ+1);%force applied by the end effector (NJ+1 st force matrix) will be zero
torque = zeros(3,1,NJ+1);%torque applied by the end effector (NJ+1 st torque matrix) will be zero

for i = NJ:-1:1
    force(:,:,i) = R(:,:,i+1)*force(:,:,i+1) + inert_force(:,:,i);
    torque(:,:,i) = inert_torque(:,:,i) + R(:,:,i+1)*torque(:,:,i+1) + cross(P_com(:,:,i),inert_force(:,:,i)) + cross(P(:,:,i+1),R(:,:,i+1)*force(:,:,i+1));
end

%joint torques
for i = 1:NJ
    if flag(i) == 1
        %joint torques for revolute joints
        joint_torques(i) = torque(3,1,i);%N-m
    else
        %joint forces for prismatic joints
        joint_torques(i) = force(3,1,i);%N-m
    end
end
joint_torques
