%% Open/closed loop thurst adjustment %%

clc
clear all
close all

%% Quadcopter parameters 

% dynamics related
m=1.035 * 4/6;                                      % mass [kg]
l=0.225 ;                                           % arm length [m]
Ix= 0.0469 ;
Iy= 0.0358 ;
Iz=  0.101 * 4/6;
I_B = diag([Ix,Iy,Iz]);
I_r = 3.357e-5;                                      % rotor momemt of inertia (kg x m^2)
g = 9.8;

% aerodynamics related
b =  1.5652e-08;% * (60/(2*pi))^2 ;                  % Thrust coeffcient intially N/(rpm^2), %now N/(rev/s^2)
k =  2.0862e-10;%* (60/(2*pi))^2 ;                   % Torque Coeffcient intially Nm/(rpm^2),% now Nm/(rev/s^2)
D=diag([0.04,0.04,0]);
R=3*0.0254;                                          % propeller radius  [m]
nb=2;                                                % number of blade
A=pi*R^2;                                            % disk area

% input params to model, P
mean_wind = [2.72 , 1.75, .006]; %NED 
P.m= m;                      
P.l= l ;                           
P.Ix= Ix ;
P.Iy= Iy ;P.Iz=  Iz;
P.I_B = diag([Ix,Iy,Iz]);
P.I_r = I_r;        
P.b =  b;                                           % Thrust coeffcient  N/(rpm^2)
P.k =  k;                                           % Torque Coeffcient  Nm/(rpm^2)
P.g = g;
P.D = D;
P.mean_wind = mean_wind;


% blade params
rho = 1.15;
nr = 20;                                             % number of radial points on a blade
npsi = 60;                                           % number of azimuthal points for 2*Pi  ROTOR PLANE


%%
the0=repmat(4,[1,11]);               %absolute value of zero lift angle of attack
cla1=repmat(1.7059*pi,[1,11]);       %2-D lift curve slope
th1=(the0+[24.9844849214694,24.4885730384207,23.6542985258914,22.4816610238794,20.9706612541160,19.1212982695717,16.9335723826041,14.4074838117246,11.5430326339323,8.34021849685304,4.79904143902087])*pi/180;   %blade pitch angle in radian
c1=[7.96284784614477,11.2448599794330,13.6346682267195,15.1322722373498,15.7376722802413,15.4508682842241,14.2718602574625,12.2006482106621,9.23723208187088,5.38161196932697,0.633787695366624]*0.001;

% interpolating data for different radius location (final size of the vectors 1*nr)
th=interp1(linspace(1/11,1,11),th1,linspace(floor(0.15*nr)/nr,1,nr),'linear','extrap');  %blade pitch angle in radian
c=interp1(linspace(1/11,1,11),c1,linspace(floor(0.15*nr)/nr,1,nr),'linear','extrap');     %cord length
cla=interp1(linspace(1/11,1,11),cla1,linspace(floor(0.15*nr)/nr,1,nr),'linear','extrap');  %2-D lift curve slope extrapolated

r=linspace(floor(nr*0.01)/nr,1,nr);    % normolized radial locations
psi=linspace(0,2*pi,npsi);             % azimuth angle

maxsize=max(nr,npsi);
numvar=11;
geometry2=zeros(numvar,maxsize);

list={R,nb,A,rho,nr,npsi,th,c,cla,r,psi};

for  i=1:numvar
    
geometry2(i,1:length(list{i}))=[list{i}];

end

% global geometry
geometry = Simulink.Signal;
geometry .DataType = 'double';
geometry .Dimensions = [length(list) npsi];
geometry .Complexity = 'real';
geometry .SamplingMode = 'Sample based';
geometry .InitialValue = 'geometry2';


%% load wind
% Aug 20 2021 Wind is just at point 8 m TODO: Spatial temporal wind add

load("LES_wind.mat")
%% Controller params
% load nominal trajectory and initial states
x0=[0,0,0,1,0,0,0,-P.mean_wind,0,0,0]; %intial state
xref = [1,1,-8,1,0,0,0,-P.mean_wind];
fc0=g*m;
uref=[fc0,0,0,10]; %initial wz is required

%gains
Q=diag([10,10,10,1,1,1,1,0.1,0.1,0.1]);

F =  diag([12,12,12,.1,.1,.1,.1,0.1,0.1,0.1]);  % Terminal cost on state 
% Rg=diag([1,5,5,0.1]);
Rg=diag([5,5,5,0.1]);
G=zeros(10,3);
G(1,1)=  0.5434 ; G(2,2)= 0.4197;G(3,3)=0.1236;
W=eye(3,3);
% gama=0.75;
gama = 0; %[0,1]

P.Q= Q;
P.R =Rg;
P.G = G;
P.W= W;                   %what is W?(HAO)
P.gama = gama;
P.F = F;
%forward flight only
load("nomTraj_forward.mat")
P.nomTraj = traj_nom;

%Lissajous_trajectory
% load("lissajous_traj_pts.mat")
% P.nomTraj = lis_traj_pts;

%test
run("get_finitegain.m")

%% run the model
tic
%  sim('Quadcopter_v2_full_forwardflight.slx') %
%  sim('Quadcopter_v2_full_lqr_BEMT_closed.slx')
sim('Quadcopter_v2_full_lqr_BEMT_NEDforwardflight.slx')
% sim('Quadcopter_v2_full_lqr_BET_open.slx')
% sim('Quadcopter_v2_full_mcv_BEMT_closed.slx') %mcv
%  sim('Quadcopter_v2_full_mcv_BET_open.slx') %mcv
toc
%% Prepare the data %% 
% 
% t=ans.tout; t=t(1:10:length(t));
% state= ans.states;
% pos=state(:,1:3);vel=state(:,8:10);quat_angle=state(:,4:7); omega = state(:,11:13);
% rel_vb = ans.v_rel_b;rel_vb = rel_vb(1:10:end,:);
% %%
% controller_torque = ans.control_force_torque; 
% controller_RPM = ans.rotor_RPM; 
% bemt_torque = ans.control_force_adjusted;
% bemt_RPM = ans.rotor_RPM_adjusted; 
% bemt_RPM =bemt_RPM(1:10:end,:);
% bemt_torque =bemt_torque(1:10:end,:);
% local_param =ans.local_param;local_param = local_param(1:4:end,:);
% local_param = local_param(1:10:end,:);

%%
% figure()
% plot(t,controller_torque(:,1))
% hold on
% plot(t, bemt_torque(:,1))
% xlabel("time, [s]")
% ylabel("Cumulative thrust, [N]")
% grid on
% legend("Controller", "BMET")
% 
%% save the data
% data_lqr_con_open_bemt_sep9 = [t , state , controller_torque ,  controller_RPM, bemt_torque ,bemt_RPM , local_param, rel_vb];
% 
% save("data_lqr_con_open_bemt_sep9.mat","data_lqr_con_open_bemt_sep9")

% 
% 
%%
% figure()
% plot(t,controller_torque(:,1))
% hold on
% grid on
% plot(t,bemt_torque(:,1))
% xlabel("time [s]")
% ylabel("thrust ")
% legend("Controller zero","BEMT zero")
% title("Thrust comparision")
% 
% figure()
% plot(t,controller_torque(:,2))
% hold on
% grid on
% plot(t,bemt_torque(:,2))
% xlabel("time [s]")
% ylabel("x ")
% legend("Controller zero","BEMT zero")
% title("torque comparision")
% 
% figure()
% plot(t,controller_torque(:,3))
% hold on
% grid on
% plot(t,bemt_torque(:,3))
% xlabel("time [s]")
% ylabel("y ")
% legend("Controller zero","BEMT zero")
% title("torque comparision")
% 
% figure()
% plot(t,controller_torque(:,4))
% hold on
% grid on
% plot(t,bemt_torque(:,4))
% xlabel("time [s]")
% ylabel("z ")
% legend("Controller zero","BEMT zero")
% title("Torque comparision")