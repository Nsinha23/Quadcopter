clc;
addpath 'C:\Users\sinha\Documents\MATLAB\Robotics\rvctools\rvctools'
startup_rvc

%% Part #2: Use Inv Kinematics to find joint angles of the robot
%------------------------------------------------------------
%Serial Link Structure of Robot
%------------------------------------------------------------
L1=Link([0 20 0 pi/2],'standard');     % base
L2=Link([0 30 60 0],'standard');       % shoulder
L3=Link([0 0 50 0],'standard');        % elbow
L4=Link([0 0 40 0],'standard');        % tilt

%------------------------------------------------------------
%Home Position = [pi/2 pi/2 0 0] - robot along x and pointing up
%End-effector will be at [30, 0, 170]
%------------------------------------------------------------

r = SerialLink([L1 L2 L3 L4]);
T=[0 0 1 30;0 -1 0 0;1 0 0 120;0 0 0 1];    % home position
guess = [pi/2 pi/2 pi/4 -pi];               % angles guessed

%-------------------------------------------------------------
%Initial Parameters
%-------------------------------------------------------------
M = [1 1 1 1 0 0];                    % degree of freedom
x1=[]; x2=[]; x3=[];x4=[];            % store data 

x_data = (30:-.5:0)';
y_data = (30^2- x_data.^2).^0.5;

x = x_data;
y = y_data;

% x = out.xdata(:,2);                       % x data exctracted
% y = out.ydata(:,2);                        % y data exctracted
z = 120 + zeros(1,size(x,1));

%--------------------------------------------------------------
%Find inverse kinematic solution
%--------------------------------------------------------------
for n =1:size(x)
    Ti=r.ikine(T,guess,M,'tol',0.001,'pinv','alpha',1.0);
    x1(n)=Ti(1)*(180/pi);
    x2(n)=Ti(2)*(180/pi);
    x3(n)=Ti(3)*(180/pi);
    x4(n) = Ti(4)*(180/pi);
    guess = [Ti(1) Ti(2) Ti(3) Ti(4)];
    T(1,4)=x(n);T(2,4)=y(n);T(3,4) = z(n);
end

% disp(x1(1));disp(x2(1));disp(x3(1));disp(x4(1));
%----------------------------------------------------------------
%Plot trajectory and joint angles from inverse kinematics
%----------------------------------------------------------------
subplot(1,3,1)
plot3(x,y,z,'LineWidth',2.5);
title('Path of quadcopter in x-y plane');
xlabel('x'); ylabel('y');zlabel('height(z)');
xlim([0 35]);ylim([0 35]);zlim([119.99 120.01]);

subplot(1,3,2)
plot(x1,'LineWidth',0.5);hold on;
plot(x2,'LineWidth',0.5); hold on;
plot(x3,'LineWidth',0.5); hold on;
plot(x4,'LineWidth',0.5); 
title('Joint angles of the robot');
legend('joint 1','joint 2','joint 3','joint 4','Location','northwest');
xlabel('# of points on the quadcopter path');ylabel('joint angle (degrees)');
set(gca,'ytick',(-210:30:210));
grid;

%% Part #3: Use Fwd Kinematics to verify the solution
%--------------------------------------------------------------------
%Use joint angles and fwd kinematics to find end-effector pos in base frame
%---------------------------------------------------------------------
% Q = [30;90;0;180].*pi/180;
Q_fwd = [x1; x2; x3; x4]'.*(pi/180);
x_fwd=[];y_fwd=[];z_fwd=[];

for i=1:size(Q_fwd)
    T = r.fkine(Q_fwd(i,:));
%     if (i==1) disp(T); end
    x_fwd(i) = T(1,4);
    y_fwd(i) = T(2,4);
    z_fwd(i) = T(3,4);
end
% T_new = r.fkine(Q);
% disp(T_new);
subplot(1,3,3)
plot3(x_fwd,y_fwd,z_fwd,'LineWidth',2.5,'color','r');
title('Retraced path using fwd kinematics');
xlim([0 35]);ylim([0 35]);zlim([119.99 120.01]);
xlabel('x');ylabel('y');zlabel('height(z)');
%==================================END====================================%



