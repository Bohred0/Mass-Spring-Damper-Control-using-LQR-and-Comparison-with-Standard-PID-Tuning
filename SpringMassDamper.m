%assume a spring-mass-damper system with mass 1kg, spring constant 4 N/m
%and damping coeff 0.2 N.s/m
A = [0 1;-4 -0.2];

B = [0;1];

C = eye(2);

D = 0;
sys=ss(A,B,C,D);

% Define the time vector and input signal
t = 0:0.01:10; % time from 0 to 10 seconds with 0.01s intervals
u = zeros(size(t)); % zero input signal
u(100:end) = 1; % step input starting at t=1s

[y,~] = step(sys);

% figure(1);
% subplot(2,1,1)
% plot(t,y(:,1))
% title('Displacement Response')
% ylabel('x (m)')
% grid on
% 
% subplot(2,1,2)
% plot(t,y(:,2))
% title('Velocity Response')
% ylabel('dx/dt (m/s)')
% xlabel('Time (s)')
% grid on
% 
% figure(2)
% pzmap(sys)
% title("Pole Zero Map")
% pole(sys)
% grid on
% 
% figure(3)
% sys.OutputName=["Displacement";"Velocity"]
% initial(sys,[1;0])

%trying some arbitrary pole locations and observing  

%Desired pole locations
Place_Eigs = zeros(2,4);

Place_Eigs(:,1) = [-2+6i;  -2-6i];
Place_Eigs(:,2) = [-4+4i;  -4-4i];
Place_Eigs(:,3) = [-5;      -6];
Place_Eigs(:,4) = [-15;    -20];

% Storage for gains
K = zeros(4,2);

for i = 1:4
    K(i,:) = place(A,B,Place_Eigs(:,i));
end

% Closed-loop systems
sys_pp1 = ss(A-B*K(1,:),B,C,D);
sys_pp2 = ss(A-B*K(2,:),B,C,D);
sys_pp3 = ss(A-B*K(3,:),B,C,D);
sys_pp4 = ss(A-B*K(4,:),B,C,D);

% % Display pole locations
% disp('Desired Pole Locations:')
% disp(Place_Eigs)

t = 0:0.01:10;


[y1,~] = step(sys_pp1,t);
[y2,~] = step(sys_pp2,t);
[y3,~] = step(sys_pp3,t);
[y4,~] = step(sys_pp4,t);

%% Displacement Comparison
% 
% figure
% plot(t,y(:,1),'LineWidth',1.5)
% hold on
% 
% plot(t,y1(:,1),'LineWidth',1.5)
% plot(t,y2(:,1),'LineWidth',1.5)
% plot(t,y3(:,1),'LineWidth',1.5)
% plot(t,y4(:,1),'LineWidth',1.5)
% 
% grid on
% xlabel('Time (s)')
% ylabel('Displacement x (m)')
% title('Displacement Step Response Comparison')
% 
% legend('Open Loop',...
%        'Poles [-2 \pm 6j]',...
%        'Poles [-4 \pm 4j]',...
%        'Poles [-5,-6]',...
%        'Poles [-15,-20]',...
%        'Location','best')

% % Velocity Comparison
% 
% figure
% plot(t,y(:,2),'LineWidth',1.5)
% hold on
% 
% plot(t,y1(:,2),'LineWidth',1.5)
% plot(t,y2(:,2),'LineWidth',1.5)
% plot(t,y3(:,2),'LineWidth',1.5)
% plot(t,y4(:,2),'LineWidth',1.5)
% 
% grid on
% xlabel('Time (s)')
% ylabel('Velocity dx/dt (m/s)')
% title('Velocity Step Response Comparison')
% 
% legend('Open Loop',...
%        'Poles [-2 \pm 6j]',...
%        'Poles [-4 \pm 4j]',...
%        'Poles [-5,-6]',...
%        'Poles [-15,-20]',...
%        'Location','best')
% 

% %Plotting Pole-Zero Map
% figure
% plot(real(Place_Eigs(:)),imag(Place_Eigs(:)),'x','MarkerSize',10)
% grid on
% xlabel('Real Axis')
% ylabel('Imaginary Axis')
% title('Chosen Closed-Loop Pole Locations')
 
% Defining cost matrices for state and actuation(assuming balanced controller)
Q = diag([10,1]); % State cost matrix(Displacement is weighted more heavily because the natural response is highly oscillatory)
R = 1;      % Actuation cost matrix
K_lqr_1=lqr(A,B,Q,R);

% Closed-loop system with Balanced LQR controller
sys_lqr_1 = ss(A-B*K_lqr_1,B,C,D);
eig(sys_lqr_1)

K_lqr_2=lqr(A,B,Q,10*R);
% Closed-loop system with Conservative LQR controller
sys_lqr_2 = ss(A-B*K_lqr_2,B,C,D);
eig(sys_lqr_2)

K_lqr_3=lqr(A,B,10*Q,R);
% Closed-loop system with aggressive LQR controller
sys_lqr_3 = ss(A-B*K_lqr_3,B,C,D);
eig(sys_lqr_3)

K_lqr_4=lqr(A,B,10*Q,0.1*R);
% Closed-loop system with extremely aggressive LQR controller
sys_lqr_4 = ss(A-B*K_lqr_4,B,C,D);
eig(sys_lqr_4)

[y_lqr_1,t,x] = step(sys_lqr_1, t);
[y_lqr_2, ~] = step(sys_lqr_2, t);
[y_lqr_3, ~] = step(sys_lqr_3, t);
[y_lqr_4, ~] = step(sys_lqr_4, t);

% figure
% plot(t,y(:,1),'LineWidth',1.5)
% hold on
% plot(t,y_lqr_1(:,1),'LineWidth',1.5)
% plot(t,y_lqr_2(:,1),'LineWidth',1.5)
% plot(t,y_lqr_3(:,1),'LineWidth',1.5)
% plot(t,y_lqr_4(:,1),'LineWidth',1.5)
% grid on
% xlabel('Time (s)')
% ylabel('Displacement x (m)')
% title('Displacement Step Response on LQR')
% legend('Open Loop','Balanced LQR','Conservative LQR','Aggressive LQR','Extremely Aggressive LQR')

% figure
% plot(t,y(:,2),'LineWidth',1.5)
% hold on
% plot(t,y_lqr_1(:,2),'LineWidth',1.5)
% plot(t,y_lqr_2(:,2),'LineWidth',1.5)
% plot(t,y_lqr_3(:,2),'LineWidth',1.5)
% plot(t,y_lqr_4(:,2),'LineWidth',1.5)
% grid on
% xlabel('Time (s)')
% ylabel('Velocity dx/dt (m)')
% title('Velocity Step Response on LQR')
% legend('Open Loop','Balanced LQR','Conservative LQR','Aggressive LQR','Extremely Aggressive LQR')

%Building PID controller, using transfer function 
G=tf(1,[1 0.2 4]);
C = pidtune(G,'PID');%PID Tuning
sys_pid=feedback(C*G,1);
[y_pid,t] = step(sys_pid,t);
%Comparing with our earlier LQR plot(Balanced LQR)
%Before Comparing our LQR controller with PID, we need to introduce a
%reference gain so the LQR controller can track 

C_disp=[1 0];
Nr = -1/(C_disp*((A-B*K_lqr_1)\B)); %Balanced LQR

sys_LQR_disp_1=ss(A-B*K_lqr_1,B*Nr,C_disp,D);
[y_lqr_disptrack_1,t,~]=step(sys_LQR_disp_1,t);

%Aggressive LQR Controller 
Nr_2 = -1/(C_disp*((A-B*K_lqr_3)\B));

sys_LQR_disp_2=ss(A-B*K_lqr_3,B*Nr_2,C_disp,D);
[y_lqr_disptrack_2,t,~]=step(sys_LQR_disp_2,t);
%--------------------------------------------
figure
plot(t,y_pid,'LineWidth',1.5)
hold on
plot(t,y_lqr_disptrack_1(:,1),'LineWidth',1.5)
plot(t,y_lqr_disptrack_2(:,1),'LineWidth',1.5)
title('Displacement Response Comparison')
xlabel('Time (s)')
ylabel('Displacement (m)')
legend('PID','LQR(Balanced)','LQR(Aggressive)')
grid on


%Performance Metrics
info_pid = stepinfo(sys_pid)
info_lqr = stepinfo(sys_LQR_disp_1)
info_lqr_2 = stepinfo(sys_LQR_disp_2)
