% offline computation of the gain begining the loading of the simulink
% model

%get the gain values
Q = P.Q;
R = P.R;
G = P.G;
W = P.W;
gama = P.gama;

%get linearized system at the linearization point
[A,B] = deriveLinSys(xref , uref, P);
K0=lqr(A,B,Q,R); %initial K0
[M, H ,K]=MCV_infinite(A,B,Q,R,G,W,-K0,gama);
P.K= K; %assign gains

