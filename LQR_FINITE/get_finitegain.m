% offline computation of the gain begining the loading of the simulink
% model

%get the gain values
Q = P.Q;
R = P.R;
G = P.G;
W = P.W;
gama = P.gama;

%prepare polynomial fitting eqn to get linearized point in the function
traj_nom=P.nomTraj;
t_nom=traj_nom(:,1);
nFit = 7;  %Order of polynomial fittig
nState= 10;
nInput= 4;
nTime = length(t_nom);
traj_nom=traj_nom(:,2:end);
u_nom=uref;
for i=1:nState
zFit(i,:) = polyfit(t_nom,traj_nom(:,i),nFit);
end

%Function handle for getting linearized system dynamics
linSys = @(t)deriveLinTraj(t,zFit,u_nom,P);

tol = 1e-6;  % Accuracy of ricatti propagation

Soln = trajectoryFMCV(t_nom,linSys,P,tol);
P.Soln = Soln;
