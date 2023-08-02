function Soln = trajectoryFMCV(t,linSys,P,tol)
% 
% Asma Tabassum
% Control, Robotics and Autnomous lab
% This function is used to solve the finite-horizon continuout-time minimum
% cost variance controller
%
%
% INPUTS:
%   t = monotonically increasing vector of times at which you would like
%       the gain matrix to be calculated
%   linSys = function handle for time-varying linear dynamics
%       [A, B] = linSys(t)
%   Q = state cost matrix 
%   R = input cost matrix 
%   F = final state cost matrix 
%   tol = accuracy of riccati equation propagation
%
% OUTPUTS:
%   Soln = struct array with solution at each point in t
%   Soln(i).t = t(i);
%   Soln(i).M = Riccati solution
%   Soln(i).H = Riccati solution
%   Soln(i).K = gain matrix at t(i)
%   Soln(i).S = Cost to go
%   Soln(i).E = close-loop eigen-values for system at t(i)
%
% NOTES:
%
%   J = x'Fx + Integral {x'Qx + u'Ru} dt



F=P.F;
Q=P.Q;
Rg=P.R;
gaama=P.gama;

nState = size(Q,1);
nInput = size(Rg,1);

userFun = @(t,z)rhs(t,z,linSys,P, nState);
z0 = [F; zeros(nState,nState)];
tSpan = [t(end), t(1)];

options = odeset();
options.RelTol = tol;
options.AbsTol = tol;
sol = ode45(userFun,tSpan,z0);

length(sol.x);
sol.x;
% nSoln=length(sol.x);
% Reevaluate at ts
z = deval(sol,t);
nSoln = length(t);
Soln(nSoln).t = 0;
Soln(nSoln).K = zeros(nState,nInput);
Soln(nSoln).M = zeros(nState,nState);
Soln(nSoln).H = zeros(nState,nState);
Soln(nSoln).E = zeros(nState,1);

for idx=1:nSoln
    i = nSoln-idx+1;
%     z=sol.y;
    zNow= z(:,i);
    p=reshape(zNow,[2*nState,nState]);
    Mnow=p(1:nState,:);
    Hnow=p(nState+1:2*nState,:);
    tNow = t(i);
    S = (Mnow+gaama*Hnow);
    [A,B] = linSys(tNow);
    K = -Rg\(B'*S);
    Soln(i).t = tNow;
    Soln(i).K = K;
    Soln(i).M = Mnow;
    Soln(i).H = Hnow;
    Soln(i).E = eig(A-B*K);
end

end

function dz = rhs(t,z,linSys,P,nState)
Q=P.Q;
Rg=P.R;
G=P.G;
W=P.W;
gaama=P.gama;
p=reshape(z,[2*nState,nState]);
M=p(1:nState,:);
H=p(11:2*nState,:);

%TODO
[A,B] = linSys(t);  %write function to find A and B
[dM, dH] = coupled_ricatti(A,B,G,W,Q,Rg,M,H,gaama);

dP=[dM;dH];
dz = reshape(dP,2*nState*nState,1);
end

function [dM, dH] = coupled_ricatti(A,B,G,W,Q,R,M,H,gaama)

T1=Q-M*B*inv(R)*B'*M+gaama^2*H*B*inv(R)*B'*H;

dM= -(A'*M + M*A + T1);
T2=4*M*G*W*G'*M-H*B*inv(R)*B'*M- M*B*inv(R)*B'*H-2*gaama*H*B*inv(R)*B'*H;

dH= -(A'*H + H*A + T2);
end


