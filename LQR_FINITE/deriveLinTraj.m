function [A, B] = deriveLinTraj(t,zFit, uref,P)

%Linearizes the system about the nominal trajectory. This function is used
%to turn a non-linear trajectory tracking problem into a time-varying
%linear problem.


[a,~]=size(zFit);z=[];
for i=1:a
    x=polyval(zFit(i,:),t);
    z=[z;x];
end

% [b,~]=size(uFit);
% u=[];
% for j=1:b
%     inp=polyval(uFit(j,:),t);
%     u= [u;inp];
% end
u=uref;
[A,B] = deriveLinSys(z,u,P);

end