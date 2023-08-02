clear all;close all;clc;
% This is a tool that can be used to generate and plot the multi-segment minimum snap trajectory that satisfies the following:
% 1) Passes through any number of waypoints at certain times.
% 2) Satisfies certain boundary conditions for velocity, acceleration and jerk (Initial and final velocity, acceleration and jerk).
% 
% 
% Features:
% 1) It is a straightforward package as everything is in one file and all the user needs is to enter very few inputs, then run the code.
% 2) It is general, which means that it works with  2D and 3D trajectories and can deal with any number of waypoints.
% 
% Inputs:
% 1) Positions of the waypoints (2D or 3D).
% 2) The desired time of each waypoint.
% 3) The desired initial and final velocity, acceleration and jerk.

% You can use the following part as an example on how this code works and how to input.
% You just need to enter your desired values into the following vectors and matrices, then run the code ... IT IS THAT SIMPLE!
% T=[0 2 4 6 8 10 12 14]; %Time of waypoints ... they must be in order. 
% Pos=[0 0 0;
%      1 0 1;
%      2 0 3;
%      3 0 4;
%      4 0 5;
%      5 0 5;
%      6 0 5;
%      7 0 5;]; % Positions of waypoints ... Each row represents (x y z) of a waypoint ... they must be in order.
             %If you have a 2D trajectory, then your matrix should have 2 columns instead of 3. 

T=[0 3 5 7 9 11]; %Time of waypoints ... they must be in order. 
Pos=[0 0 0;
     1 0 4;
     4 0 4;
     4 3 4;
     1 3 4;
     1 0 4;]; 
 v0=[0,0,0]; % Initial velocity (the components are in 3D (x y z)) [MUST have the same length as the number of columns of Pos].
             % If you have a 2D trajectory, then your vector size should be 1*2 NOT 1*3. 
 vf=[0,0,0]; % Final velocity (the components are in 3D (x y z)) [MUST have the same length as the number of columns of Pos].
              % If you have a 2D trajectory, then your vector size should be 1*2 NOT 1*3. 
 a0=[0,0,0]; % Initial acceleration (the components are in 3D (x y z)) [MUST have the same length as the number of columns of Pos].
              % If you have a 2D trajectory, then your vector size should be 1*2 NOT 1*3. 
 af=[0,0,0]; % Final acceleration (the components are in 3D (x y z)) [MUST have the same length as the number of columns of Pos].
              % If you have a 2D trajectory, then your vector size should be 1*2 NOT 1*3.
 j0=[0,0,0]; % Initial jerk (the components are in 3D (x y z)) [MUST have the same length as the number of columns of Pos].
              % If you have a 2D trajectory, then your vector size should be 1*2 NOT 1*3. 
 jf=[0,0,0]; % Final jerk (the components are in 3D (x y z)) [MUST have the same length as the number of columns of Pos].
              % If you have a 2D trajectory, then your vector size should be 1*2 NOT 1*3. 
              
              
% Outputs:
% 1) The simulation time vector (t) in case the user wants to perform further analysis.
% 2) The Coefficient Vector (C), which contains all the coefficients of the 5th degree polynomials of the position trajectory segments. It has a dimension (6n), where n is the number of trajectory segments. The first 6 elements are the coefficients of the first trajectory segment, the second 6 elements are for the second segment and so on so forth. for each segment, the first coefficient is associated with (t^5) and the so on until the 6th coefficient in the segment which is associated with (t^0).
% 3) The minimum snap trajectory and the corresponding trajectory for the velocity and the acceleration (PP, VV, AA) each as a matrix of size (d,n), where d is the dimension of the problem (2D or 3D) and n is the number of trajectory segments. 
% 4) The minimum snap trajectory and the corresponding trajectory for the velocity and the acceleration (POS, VEL, ACC) each as a piecewise function of time. This is a piecewise matrix of size (d,n), where d is the dimension of the problem (2D or 3D) and n is the number of trajectory segments. 
% 5) The minimum snap trajectory and the corresponding trajectory for the velocity and the acceleration (P, V, A) each as a numerical matrix. The numerical values are obtained after substituting the trajectory time into the piecewise function. This is a matrix of size (tt,n), where tt is the dimension of the simulation time vector and n is the number of trajectory segments.  
% 6) Several plot such as: Position vs time (each direction on a separate subplot), Velocity vs time (each direction on a separate subplot), Acceleration vs time (each direction on a separate subplot), The path of motion (in 2D or 3D), The velocity path of motion (in 2D or 3D), The acceleration path of motion (in 2D or 3D). 
%              
[t,Cs,PPs,VVs,AAs,POSs,VELs,ACCs,Ps,Vs,As] = MinimumSnapGenerator(T,Pos,v0,vf,a0,af,j0,jf) ;
% For the definistions of each output, please go to the preivious paragraph under the title (Outputs)
traj_pts=[t' Ps' Vs'];
save("circular_traj_pts.mat","traj_pts")

%% forward trajectory 
close all;clear all; clc;
T=[0 10 20 30 40 50 60 70 80 90 100];
Pos=[  0    0 -10;
      10   10 -10;
      20   20 -10;
      30   30 -10;
      40   40 -10;
      50   50 -10;
      60   60 -10;
      70   70 -10;
      80   80 -10;
      90   90 -10;
      100  100 -10;]; 
v0=[0,0,0];
vf=[0,0,0];
a0=[0,0,0];
af=[0,0,0];
j0=[0,0,0];
jf=[0,0,0];
quat = [ones(1, T(end)*100+1);zeros(3, T(end)*100+1);];
[t,Cs,PPs,VVs,AAs,POSs,VELs,ACCs,Ps,Vs,As] = MinimumSnapGenerator(T,Pos,v0,vf,a0,af,j0,jf);
traj_nom= [t' Ps' quat' Vs'];
traj_nom(end,:) = [];
save("forward_traj_LQR.mat","traj_nom")

mean_wind = [2.72 , 1.75, 0.006]; %NED
traj_nom(1:end,9:11) = traj_nom(1:end,9:11)- repmat(mean_wind,length(traj_nom),1); 
save("forward_traj_MCV.mat","traj_nom")
%% waypoint and horizontal flying trajectories
T=[0 10 20 30 40 50 60 70 80 90 100];
Pos=[ 0   0   0;
     10  10  -10;
     20  20  -10;
     30  30  -10;
     40  40  -10;
     50  50  -10;
     60  60  -10;
     70  70  -10;
     80  80  -10;
     90  90  -10;
     100 100 -10;]; 
v0=[0,0,0];
vf=[0,0,0];
a0=[0,0,0];
af=[0,0,0];
j0=[0,0,0];
jf=[0,0,0];
quat = [ones(1, T(end)*100+1);zeros(3, T(end)*100+1);];
[t,Cs,PPs,VVs,AAs,POSs,VELs,ACCs,Ps,Vs,As] = MinimumSnapGenerator(T,Pos,v0,vf,a0,af,j0,jf);
traj_pts=[t' Ps' quat' Vs'];
save("traj_nom_long3.mat","traj_pts")
%% hovering trajectories
T=[0 10 20 30 40 50 60 70 80 90 100];
Pos=[ 0   0   0;
      0   0  -10;
      0   0  -20;
      0   0  -30;
      0   0  -40;
      0   0  -50;
      0   0  -50;
      0   0  -50;
      0   0  -50;
      0   0  -50;
      0   0  -50;]; 
v0=[0,0,0];
vf=[0,0,0];
a0=[0,0,0];
af=[0,0,0];
j0=[0,0,0];
jf=[0,0,0];
quat = [ones(1, T(end)*100+1);zeros(3, T(end)*100+1);];
[t,Cs,PPs,VVs,AAs,POSs,VELs,ACCs,Ps,Vs,As] = MinimumSnapGenerator(T,Pos,v0,vf,a0,af,j0,jf);
traj_pts=[t' Ps' quat' Vs'];
mean_wind = [2.72 , 1.75, -0.006];
traj_pts(:,end-2:end) =traj_pts(:,end-2:end)- ones(10001,1)*mean_wind;
save("hovering_traj.mat","traj_pts")
%% generate the lissajous 0 trajectories
T = 0:1:100;
A1_x = 1.5; A1_y =1.5;A1_z =1.5;n1_x =0.2/2;n1_y =0.2/2;n1_z =0.3/2;
A2_x = 0.02; A2_y =0.02;A2_z =0;n2_x =1.2;n2_y =1.2;n2_z =1.8;
Pos = [A1_x*(1-cos(2*pi*n1_x*T)) + A2_x*(1-cos(2*pi*n2_x*T));
       A1_y*sin(2*pi*n1_y*T)     +     A2_y*sin(2*pi*n2_y*T);
       A1_z*sin(2*pi*n1_z*T)     +     A2_z*sin(2*pi*n2_z*T);]';
v0 = zeros(1, 3);
vf = zeros(1, 3);
a0 = zeros(1, 3);
af = zeros(1, 3);
j0 = zeros(1, 3);
jf = zeros(1, 3);
quat = [ones(1, T(end)*100+1);zeros(3, T(end)*100+1);];
[t,Cs,PPs,VVs,AAs,POSs,VELs,ACCs,Ps,Vs,As] = MinimumSnapGenerator(T,Pos,v0,vf,a0,af,j0,jf) ;
% traj=[t' Ps' quat' Vs'];
% save("lissajous_traj_pts.mat","traj_norm")

traj=[t' Ps' quat' Vs'];
traj(end,:)  = [];
traj_nom = [traj;100 0 0 0 1 0 0 0 0 0 0];
save("lissajous_traj_pts.mat","traj_nom")
%% generate the lissajous 1 trajectories
T = 0:1:100;
A1_x = 1.5; A1_y =1.5;A1_z =1.5;n1_x =0.2/2;n1_y =0.2/2;n1_z =0.3/2;
Pos = [A1_x*(1-cos(2*pi*n1_x*T));
       A1_y*sin(2*pi*n1_y*T);
       A1_z*sin(2*pi*n1_z*T);]';
v0 = zeros(1, 3);
vf = zeros(1, 3);
a0 = zeros(1, 3);
af = zeros(1, 3);
j0 = zeros(1, 3);
jf = zeros(1, 3);
quat = [ones(1, T(end)*100+1);zeros(3, T(end)*100+1);];
[t,Cs,PPs,VVs,AAs,POSs,VELs,ACCs,Ps,Vs,As] = MinimumSnapGenerator(T,Pos,v0,vf,a0,af,j0,jf) ;
% traj=[t' Ps' quat' Vs'];
% save("lissajous_traj_pts.mat","traj_norm")

traj=[t' Ps' quat' Vs'];
traj(end,:)  = [];
traj_nom = [traj;100 0 0 0 1 0 0 0 0 0 0];
save("lissajous_traj_pts.mat","traj_nom")

%% generate the lissajous 2 trajectories
T = 0:1:100;
A1_x = 15; A1_y =15;A1_z =15;n1_x =0.2/4;n1_y =0.2/4;n1_z =0.1/4;
A2_x = 0.02; A2_y =0.02;A2_z =0;n2_x =1.2;n2_y =1.2;n2_z =1.8;
Pos = [A1_x*(1-cos(2*pi*n1_x*T)) + A2_x*(1-cos(2*pi*n2_x*T));
       A1_y*sin(2*pi*n1_y*T)     +     A2_y*sin(2*pi*n2_y*T);
       A1_z*sin(2*pi*n1_z*T)     +     A2_z*sin(2*pi*n2_z*T);]';
v0 = zeros(1, 3);
vf = zeros(1, 3);
a0 = zeros(1, 3);
af = zeros(1, 3);
j0 = zeros(1, 3);
jf = zeros(1, 3);
quat = [ones(1, T(end)*100+1);zeros(3, T(end)*100+1);];
[t,Cs,PPs,VVs,AAs,POSs,VELs,ACCs,Ps,Vs,As] = MinimumSnapGenerator(T,Pos,v0,vf,a0,af,j0,jf) ;
% traj=[t' Ps' quat' Vs'];
% save("lissajous_traj_pts.mat","traj_norm")

traj=[t' Ps' quat' Vs'];
traj(end,:)  = [];
traj_nom = [traj;100 0 0 0 1 0 0 0 0 0 0];
save("lissajous_traj_pts.mat","traj_nom")

%% generate the lissajous 3 trajectories
T = 0:1:100;
A1_x = 15; A1_y =15;A1_z =15;n1_x =0.2/4;n1_y =0.2/4;n1_z =0.1/4;
Pos = [A1_x*(1-cos(2*pi*n1_x*T)) ;
       A1_y*sin(2*pi*n1_y*T);
       A1_z*sin(2*pi*n1_z*T);]';
v0 = zeros(1, 3);
vf = zeros(1, 3);
a0 = zeros(1, 3);
af = zeros(1, 3);
j0 = zeros(1, 3);
jf = zeros(1, 3);
quat = [ones(1, T(end)*100+1);zeros(3, T(end)*100+1);];
[t,Cs,PPs,VVs,AAs,POSs,VELs,ACCs,Ps,Vs,As] = MinimumSnapGenerator(T,Pos,v0,vf,a0,af,j0,jf) ;
% traj=[t' Ps' quat' Vs'];
% save("lissajous_traj_pts.mat","traj_norm")

traj=[t' Ps' quat' Vs'];
traj(end,:)  = [];
traj_nom = [traj;100 0 0 0 1 0 0 0 0 0 0];
save("lissajous_traj_pts.mat","traj_nom")