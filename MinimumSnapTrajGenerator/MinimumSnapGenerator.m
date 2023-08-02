function [t,Cs,PPs,VVs,AAs,POSs,VELs,ACCs,Ps,Vs,As]=MinimumSnapGenerator(T,Pos,v0,vf,a0,af,j0,jf)   
%% Prelimenary Manupulations  
[rows,columns]=size(Pos);  %rows= number of waypoints ... columns= number of coordinates (1D, 2D or 3D) 
I=eye(columns);
n=rows-1;  %n= number of trajectories 

%% Waypoints' Matrix
R=zeros(8*columns*n,8*columns*n); % Start with zero matrix for R

% Positions of all waypoints
for i=1:1:n
     R(((1+(2*columns*(i-1))):(columns+(2*columns*(i-1)))),((1+(8*columns*(i-1))):(8*columns+(8*columns*(i-1)))))=[I*T(i)^7 I*T(i)^6 I*T(i)^5 I*T(i)^4 I*T(i)^3 I*T(i)^2 I*T(i)^1 I];
     R(((1+columns+(2*columns*(i-1))):(2*columns+(2*columns*(i-1)))),((1+(8*columns*(i-1))):(8*columns+(8*columns*(i-1)))))=[I*T(i+1)^7 I*T(i+1)^6 I*T(i+1)^5 I*T(i+1)^4 I*T(i+1)^3 I*T(i+1)^2 I*T(i+1)^1 I]; 
end     
% Velocity boundary conditions (inital and final waypoints)        
R(((1+(2*columns*n)):(columns+(2*columns*n))),(1:8*columns)) =[7*I*T(1)^6 6*I*T(1)^5 5*I*T(1)^4 4*I*T(1)^3 3*I*T(1)^2 2*I*T(1)^1 I zeros(columns,columns)];
R(((1+columns+(2*columns*n)):((2*columns)+(2*columns*n))),((1+(8*columns*(n-1))):(8*columns*n)))=[7*I*T(n+1)^6 6*I*T(n+1)^5 5*I*T(n+1)^4 4*I*T(n+1)^3 3*I*T(n+1)^2 2*I*T(n+1)^1 I zeros(columns,columns)];
% Equal Accelaration boundary conditions (initial and final waypoints)
R(((1+(2*columns)+(2*columns*n)):((3*columns)+(2*columns*n))),(1:8*columns)) =[42*I*T(1)^5 30*I*T(1)^4 20*I*T(1)^3 12*I*T(1)^2 6*I*T(1)^1 2*I zeros(columns,columns) zeros(columns,columns)];
R(((1+(3*columns)+(2*columns*n)):((4*columns)+(2*columns*n))),((1+(8*columns*(n-1))):(8*columns*n)))=[42*I*T(n+1)^5 30*I*T(n+1)^4 20*I*T(n+1)^3 12*I*T(n+1)^2 6*I*T(n+1)^1 2*I zeros(columns,columns) zeros(columns,columns)];
% Equal Jerk boundary conditions (initial and final waypoints)
R(((1+(4*columns)+(2*columns*n)):((5*columns)+(2*columns*n))),(1:8*columns)) =[210*I*T(1)^4 120*I*T(1)^3 60*I*T(1)^2 24*I*T(1)^1 6*I zeros(columns,columns) zeros(columns,columns) zeros(columns,columns)];
R(((1+(5*columns)+(2*columns*n)):((6*columns)+(2*columns*n))),((1+(8*columns*(n-1))):(8*columns*n)))=[210*I*T(n+1)^4 120*I*T(n+1)^3 60*I*T(n+1)^2 24*I*T(n+1)^1 6*I zeros(columns,columns) zeros(columns,columns) zeros(columns,columns)];
%Equal velocity, accelaration , jerk, snap, crackle and pop at intermideate waypoints
for i=1:n-1
R(((1+(columns*(i-1))+(6*columns)+(2*columns*n)):(columns+(columns*(i-1))+(6*columns)+(2*columns*n))),(1+(8*columns*(i-1)):8*(i+1)*columns)) =[7*I*T(i+1)^6 6*I*T(i+1)^5 5*I*T(i+1)^4 4*I*T(i+1)^3 3*I*T(i+1)^2 2*I*T(i+1)^1 I zeros(columns,columns) -7*I*T(i+1)^6 -6*I*T(i+1)^5 -5*I*T(i+1)^4 -4*I*T(i+1)^3 -3*I*T(i+1)^2 -2*I*T(i+1)^1 -I zeros(columns,columns)]; % Equal velocity at intermediate waypoints
R(((1+(columns*(i-1))+(columns*(n-1))+(6*columns)+(2*columns*n)):(columns+(columns*(i-1))+(columns*(n-1))+(6*columns)+(2*columns*n))),(1+(8*columns*(i-1)):8*(i+1)*columns)) =[42*I*T(i+1)^5 30*I*T(i+1)^4 20*I*T(i+1)^3 12*I*T(i+1)^2 6*I*T(i+1)^1 2*I zeros(columns,columns) zeros(columns,columns) -42*I*T(i+1)^5 -30*I*T(i+1)^4 -20*I*T(i+1)^3 -12*I*T(i+1)^2 -6*I*T(i+1)^1 -2*I zeros(columns,columns) zeros(columns,columns)]; % Equal acceleration at intermediate waypoints
R(((1+(columns*(i-1))+(2*columns*(n-1))+(6*columns)+(2*columns*n)):(columns+(columns*(i-1))+(2*columns*(n-1))+(6*columns)+(2*columns*n))),(1+(8*columns*(i-1)):8*(i+1)*columns)) =[210*I*T(i+1)^4 120*I*T(i+1)^3 60*I*T(i+1)^2 24*I*T(i+1)^1 6*I zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) -210*I*T(i+1)^4 -120*I*T(i+1)^3 -60*I*T(i+1)^2 -24*I*T(i+1)^1 -6*I zeros(columns,columns) zeros(columns,columns) zeros(columns,columns)]; % Equal jerk at intermediate waypoints
R(((1+(columns*(i-1))+(3*columns*(n-1))+(6*columns)+(2*columns*n)):(columns+(columns*(i-1))+(3*columns*(n-1))+(6*columns)+(2*columns*n))),(1+(8*columns*(i-1)):8*(i+1)*columns)) =[840*I*T(i+1)^3 360*I*T(i+1)^2 120*I*T(i+1)^1 24*I zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) -840*I*T(i+1)^3 -360*I*T(i+1)^2 -120*I*T(i+1)^1 -24*I zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) zeros(columns,columns)]; % Equal snap at intermediate waypoints
R(((1+(columns*(i-1))+(4*columns*(n-1))+(6*columns)+(2*columns*n)):(columns+(columns*(i-1))+(4*columns*(n-1))+(6*columns)+(2*columns*n))),(1+(8*columns*(i-1)):8*(i+1)*columns)) =[2520*I*T(i+1)^2 720*I*T(i+1)^1 120*I zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) -2520*I*T(i+1)^2 -720*I*T(i+1)^1 -120*I zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) zeros(columns,columns)]; % Equal crackle at intermediate waypoints
R(((1+(columns*(i-1))+(5*columns*(n-1))+(6*columns)+(2*columns*n)):(columns+(columns*(i-1))+(5*columns*(n-1))+(6*columns)+(2*columns*n))),(1+(8*columns*(i-1)):8*(i+1)*columns)) =[5040*I*T(i+1)^1 720*I zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) -5040*I*T(i+1)^1 -720*I zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) zeros(columns,columns)]; % Equal pop at intermediate waypoints
end

%% Boundary Conditions Matrix
BC=zeros(8*columns*n,1);
BC(1:columns,1:1)=(Pos(1,:)).'; % Position of the first waypoint
PosInter=[];
for i=2:1:n
    posinter=[Pos(i,:);Pos(i,:)];
    PosInter=[PosInter;posinter];
end
for i=1:1:2*(n-1)
    BC((1+columns+((i-1)*columns)):(columns+columns+((i-1)*columns)),1)=(PosInter(i,:)).';
end
BC(((1+columns+(2*(n-1)*columns)):(columns+columns+(2*(n-1)*columns))),1)=(Pos(end,:)).'; % Position of the final waypoint
BC(((1+(2*columns)+(2*(n-1)*columns)):(columns+(2*columns)+(2*(n-1)*columns))),1)=(v0).'; %initial velocity
BC(((1+(3*columns)+(2*(n-1)*columns)):(columns+(3*columns)+(2*(n-1)*columns))),1)=(vf).'; %final velocity
BC(((1+(4*columns)+(2*(n-1)*columns)):(columns+(4*columns)+(2*(n-1)*columns))),1)=(a0).'; %initial acceleration
BC(((1+(5*columns)+(2*(n-1)*columns)):(columns+(5*columns)+(2*(n-1)*columns))),1)=(af).'; %final acceleration
BC(((1+(6*columns)+(2*(n-1)*columns)):(columns+(6*columns)+(2*(n-1)*columns))),1)=(j0).'; %initial jerk
BC(((1+(7*columns)+(2*(n-1)*columns)):(columns+(7*columns)+(2*(n-1)*columns))),1)=(jf).'; %final jerk

%% Coefficient  Vector
Cs=R\BC;

%% Trajectory Ploynomial
syms t

PPs=sym(zeros(columns,n));
for i=1:1:n
  PPs(1:columns,i)=(Cs(((1+(8*(i-1)*columns)):(columns+(8*(i-1)*columns))),1)*t^7)+(Cs(((1+columns+(8*(i-1)*columns)):(2*columns+(8*(i-1)*columns))),1)*t^6)+(Cs(((1+2*columns+(8*(i-1)*columns)):(3*columns+(8*(i-1)*columns))),1)*t^5)+(Cs(((1+3*columns+(8*(i-1)*columns)):(4*columns+(8*(i-1)*columns))),1)*t^4)+(Cs(((1+4*columns+(8*(i-1)*columns)):(5*columns+(8*(i-1)*columns))),1)*t^3)+(Cs(((1+5*columns+(8*(i-1)*columns)):(6*columns+(8*(i-1)*columns))),1)*t^2)+(Cs(((1+6*columns+(8*(i-1)*columns)):(7*columns+(8*(i-1)*columns))),1)*t)+(Cs(((1+7*columns+(8*(i-1)*columns)):(8*columns+(8*(i-1)*columns))),1));
end

PPs=vpa(PPs);  % Minimal Jerk Trajectory ... Position ... Each COLUMN represents one of the trajectories between two points ... Number of columns = n
VVs=vpa(diff(PPs,t)); % Minimal Jerk Trajectory ... Velocity ... Each COLUMN represents one of the trajectories between two points ... Number of columns = n
AAs=vpa(diff(VVs,t)); % Minimal Jerk Trajectory ... Acceleration ... Each COLUMN represents one of the trajectories between two points ... Number of columns = n

POSs=piecewise(T(1)<=t<T(2),PPs(:,1));
if n>1
for i=2:1:n
POSs=piecewise(T(i)<=t<T(i+1),PPs(:,i),POSs);   %Piecewise POSITION trajectory (TIME function) ... REMOVE the semicolon if you want to display
end
end

VELs=piecewise(T(1)<=t<T(2),VVs(:,1));
if n>1
for i=2:1:n
VELs=piecewise(T(i)<=t<T(i+1),VVs(:,i),VELs);  %Piecewise VELOCITY trajectory (TIME function) ... REMOVE the semicolon if you want to display
end
end

ACCs=piecewise(T(1)<=t<T(2),AAs(:,1));
if n>1
for i=2:1:n
ACCs=piecewise(T(i)<=t<T(i+1),AAs(:,i),ACCs); %Piecewise ACCELERATION trajectory (TIME function) ... REMOVE the semicolon if you want to display 
end
end
 
t=T(1):1e-2:T(n+1);
Ps=double(subs(POSs)); %POSITION trajectory (NUMERICAL VALUES AFTER SUB TIME) ... REMOVE the semicolon if you want to display 
Vs=double(subs(VELs)); %VELOCITY trajectory (NUMERICAL VALUES AFTER SUB TIME) ... REMOVE the semicolon if you want to display
As=double(subs(ACCs)); %ACCELERATION trajectory (NUMERICAL VALUES AFTER SUB TIME) ... REMOVE the semicolon if you want to display
%% Ploting

figure('WindowState', 'maximized')
for i=1:1:columns
subplot(columns,1,i)
txt=['Position in D_',num2str(i),' (m)' ];
plot(t,Ps(i,:),'DisplayName',txt,'LineWidth',3)
grid on
legend show;
ylabel(txt);
xlabel('Time (sec)')
title(txt);
set(gca,'FontSize',15);
end

figure('WindowState', 'maximized')
for i=1:1:columns
subplot(columns,1,i)
txt=['Velocity in D_',num2str(i),' (m/sec)' ];
plot(t,Vs(i,:),'DisplayName',txt,'LineWidth',3)
grid on
legend show;
ylabel(txt);
xlabel('Time (sec)')
title(txt);
set(gca,'FontSize',15);
end

figure('WindowState', 'maximized')
for i=1:1:columns
subplot(columns,1,i)
txt=['Acceleration in D_',num2str(i),' (m/sec^2)' ];
plot(t,As(i,:),'DisplayName',txt,'LineWidth',3)
grid on
legend show;
ylabel(txt);
xlabel('Time (sec)')
title(txt);
set(gca,'FontSize',15);
end

figure('WindowState', 'maximized')
if columns==2
  plot(Ps(1,:),Ps(2,:),'LineWidth',3) 
  grid on
  xlabel('D_1')
  ylabel('D_2')
  title('Path')
  set(gca,'FontSize',15);
elseif columns==3
  plot3(Ps(1,:),Ps(2,:),Ps(3,:),'LineWidth',3) 
  grid on
  xlabel('D_1')
  ylabel('D_2')
  zlabel('D_3')
  title('Path')
  set(gca,'FontSize',15);
end

figure('WindowState', 'maximized')
if columns==2
  plot(Vs(1,:),Vs(2,:),'LineWidth',3) 
  grid on
  xlabel('D_1')
  ylabel('D_2')
  title('Velocity Path')
  set(gca,'FontSize',15);
elseif columns==3
  plot3(Vs(1,:),Vs(2,:),Vs(3,:),'LineWidth',3) 
  grid on
  xlabel('D_1')
  ylabel('D_2')
  zlabel('D_3')
  title('Velocity Path')
  set(gca,'FontSize',15);
end

figure('WindowState', 'maximized')
if columns==2
  plot(As(1,:),As(2,:),'LineWidth',3) 
  grid on
  xlabel('D_1')
  ylabel('D_2')
  title('Acceleration Path')
  set(gca,'FontSize',15);
elseif columns==3
  plot3(As(1,:),As(2,:),As(3,:),'LineWidth',3) 
  grid on
  xlabel('D_1')
  ylabel('D_2')
  zlabel('D_3')
  title('Acceleration Path')
  set(gca,'FontSize',15);
end
