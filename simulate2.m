clear; clc; close all;

workspace = [0,2*(sqrt(2))];

% Robot link lengths
L1 = 2*(sqrt(2));
L2 = 2*(sqrt(2));
L3 = 0.5;
dx = 0.5;
dy = 0.5;
dz = 0.5;
New_dx = 4;
New_dy = 4;
New_dz = 4;
%recomendation workspace
% 0.5 < dx < 4.5
% 0.5 < dy < 4
% 0.5 < dz < 4
% ส่วนstatic ดูแรง
Fx =0;
Fy =0;
Fz =10;

Mx = 0;
My = 0;
Mz = 0;

W = [Fx; Fy; Fz; Mx; My; Mz];
F = [Fx; Fy; Fz];
PZ3 = zeros(3,300);
PY3 = zeros(3,300);
PX3 = zeros(3,300);

F_scale = 0.3;     

% center = [dx,dy,dz];
% new_center = [New_dx,New_dy,New_dz];
% Time vector

t = linspace(0, 6, 300);
DZ = ((New_dz -dz)*t/6)+dz;
DX = ((New_dx -dx)*t/6)+dx;
DY = ((New_dy -dy)*t/6)+dy;

q1x = zeros(1,300);
q2x = zeros(1,300);
q3x = zeros(1,300);
q1y = zeros(1,300);
q2y = zeros(1,300);
q3y = zeros(1,300);
q1z = zeros(1,300);
q2z = zeros(1,300);
q3z = zeros(1,300);

for i = 1:length(t)

    q2x(i) =  Invertq2(L1,L2,L3,DZ(i),DY(i),0);
    q1x(i) =  Invertq1(L1,L2,L3,DZ(i),DY(i),q2x(i),0);
    q3x(i) =  Invertq3(q1x(i),q2x(i),0);
    thetaX1 = q1x;
    thetaX2 = q2x;
    thetaX3 = q3x;

    q2y(i)  = Invertq2(L1,L2,L3,DZ(i),DX(i),0);
    q1y(i)  = Invertq1(L1,L2,L3,DZ(i),DX(i),q2y(i) ,0);
    q3y(i)  = Invertq3(q1y(i) ,q2y(i) ,0);
    thetaY1 = q1y;
    thetaY2 = q2y;
    thetaY3 = q3y;
    
    q2z(i) = Invertq2(L1,L2,L3,-(5-DX(i)),-DY(i),180);
    q1z(i) = Invertq1(L1,L2,L3,-(5-DX(i)),-DY(i),q2z(i),180);
    q3z(i) = Invertq3(q1z(i),q2z(i),180);
    thetaZ1 = q1z;
    thetaZ2 = q2z;
    thetaZ3 = q3z;
end
qx = [rad2deg(q1x(1)),rad2deg(q1x(300)),rad2deg(q2x(1)),rad2deg(q2x(300)),rad2deg(q3x(1)),rad2deg(q3x(300))];
qy = [rad2deg(q1y(1)),rad2deg(q1y(300)),rad2deg(q2y(1)),rad2deg(q2y(300)),rad2deg(q3y(1)),rad2deg(q3y(300))];
qz = [rad2deg(q1z(1)),rad2deg(q1z(300)),rad2deg(q2z(1)),rad2deg(q2z(300)),rad2deg(q3z(1)),rad2deg(q3z(300))];
% Setup figure
figure('Color','white');
hold on; grid on; axis equal;
xlabel('Z'); ylabel('X'); zlabel('Y');
title('tripteron Robot Simulation');

xlim([-2*(sqrt(2)) 6]); ylim([-2*(sqrt(2)) 6+2*(sqrt(2))]); zlim([-2*(sqrt(2)) 6]);

view(90,0);   % 3D camera angle

trace = animatedline('Color','m','LineWidth',1.5);
% 
% % ===== force pointer =====
% EE_now = [0 0 0];
% F_arrow = quiver3(EE_now(1),EE_now(2),EE_now(3), ...
%                   0,0,0, 'LineWidth',2,'Color','r','MaxHeadSize',2);
% 
for k = 1:length(t)

  %DH parameter
    T0X = fowardx(0,DX(k),0, 0);
    T01X = T0X * fowardx(0, 0,0, thetaX1(k));          % Joint 1 rotates X
    T02X = T01X * fowardx(L1, 0,0, thetaX2(k));  
    T03X = T02X * fowardx(L2, 0, 0, thetaX3(k)); 
    T0EX = T03X * fowardx(L3, 0, 0, 0); 


    T0Y = fowardy(0, DY(k),0, 0);
    T01Y = T0Y * fowardy(0, 0,0, thetaY1(k));          % Joint 1 rotates y
    T02Y = T01Y * fowardy(L1, 0,0, thetaY2(k));  
    T03Y = T02Y * fowardy(L2, 0, 0, thetaY3(k)); 
    T0EY = T03Y * fowardy(L3, 0, 0, 0);

    T0Z = fowardz(0, DZ(k),0, 0);
    T01Z = T0Z * fowardz(0, 0,0, thetaZ1(k));          % Joint 1 rotates Z
    T02Z = T01Z * fowardz(L1, 0,0, thetaZ2(k) );  
    T03Z = T02Z * fowardz(L2, 0, 0, thetaZ3(k) ); 
    T0EZ = T03Z * fowardz(L3, 0, 0, 0); 

    %joint position


    pX1 = T01X(1:3,4);
    pX2 = T02X(1:3,4);
    pX3 = T03X(1:3,4);
    pX4 = T0EX(1:3,4);

    pY1 = T0Y(1:3,4);
    pY2 = T02Y(1:3,4);
    pY3 = T03Y(1:3,4);
    pY4 = T0EY(1:3,4);

    pZ1 = T0Z(1:3,4);
    pZ2 = T02Z(1:3,4);
    pZ3 = T03Z(1:3,4);
    pZ4 = T0EZ(1:3,4);
    pZ1(1)=pZ1(1)+5;
    pZ2(1)=pZ2(1)+5;
    pZ3(1)=pZ3(1)+5;
    pZ4(1)=pZ4(1)+5;

    PZ3(:,k) = pZ3;
    PX3(:,k) = pX3;
    PY3(:,k) = pY3;

     cla;
     % Draw robot links (thick 3D lines)
    plot3([0 0], [0 0], [0 5], 'k', 'LineWidth',2);
    plot3([0 0], [0 5], [0 0], 'k', 'LineWidth',2);
    plot3([0 5], [5 5], [0 0], 'k', 'LineWidth',2);

    plot3([pX1(3) pX2(3)], [pX1(1) pX2(1)], [pX1(2) pX2(2)], 'r', 'LineWidth',2);
    plot3([pX2(3) pX3(3)], [pX2(1) pX3(1)], [pX2(2) pX3(2)], 'r', 'LineWidth',2);
    plot3([pX3(3) pX4(3)], [pX3(1) pX4(1)], [pX3(2) pX4(2)], 'y', 'LineWidth',4);
    % 
    % 
    plot3([pY1(3) pY2(3)], [pY1(1) pY2(1)], [pY1(2) pY2(2)], 'g', 'LineWidth',2);
    plot3([pY2(3) pY3(3)], [pY2(1) pY3(1)], [pY2(2) pY3(2)], 'g', 'LineWidth',2);
    plot3([pY3(3) pY4(3)], [pY3(1) pY4(1)], [pY3(2) pY4(2)], 'y', 'LineWidth',4);
    % 
    plot3([pZ1(3) pZ2(3)], [pZ1(1) pZ2(1)], [pZ1(2) pZ2(2)], 'B', 'LineWidth',2);
    plot3([pZ2(3) pZ3(3)], [pZ2(1) pZ3(1)], [pZ2(2) pZ3(2)], 'b', 'LineWidth',2);
    plot3([pZ3(3) pZ4(3)], [pZ3(1) pZ4(1)], [pZ3(2) pZ4(2)], 'y', 'LineWidth',4);

    % plot3([pZ3(1) pY3(1)], [pZ3(2) pY3(2)], [pZ3(3) pY3(3)], 'y', 'LineWidth',4);
    % plot3([pX3(1) pY3(1)], [pX3(2) pY3(2)], [pX3(3) pY3(3)], 'y', 'LineWidth',4);

    % Draw joints as spheres
    scatter3(pX1(3), pX1(1), pX1(2), 80, 'filled','r');
    scatter3(pX2(3), pX2(1), pX2(2), 80, 'filled','r');
    scatter3(pX3(3), pX3(1), pX3(2), 80, 'filled','r');
    scatter3(pX4(3), pX4(1), pX4(2), 120, 'filled', 'y');
    scatter3(pY1(3), pY1(1), pY1(2), 80, 'filled','g');
    scatter3(pY2(3), pY2(1), pY2(2), 80, 'filled','g');
    scatter3(pY3(3), pY3(1), pY3(2), 80, 'filled','g');
    scatter3(pY4(3), pY4(1), pY4(2), 120, 'filled', 'y');
    scatter3(pZ1(3), pZ1(1), pZ1(2), 80, 'filled','b');
    scatter3(pZ2(3), pZ2(1), pZ2(2), 80, 'filled','b');
    scatter3(pZ3(3), pZ3(1), pZ3(2), 80, 'filled','b');
    scatter3(pZ4(3), pZ4(1), pZ4(2), 120, 'filled', 'y');


    % ===== pointer force yello =====
    % EE_now = (pZ3 + pX3 + pY3) / 3;
    % 
    % ForceTip = EE_now + F_scale * F;     % ปลายแท่งแรง
    % 
    % plot3([EE_now(1) ForceTip(1)], ...
    %       [EE_now(2) ForceTip(2)], ...
    %       [EE_now(3) ForceTip(3)], ...
    %       'Color','y','LineWidth',3);




    % camorbit(0.3, 0);
    drawnow limitrate; 
% 
end



%% ==================== Cal ====================

dt = t(300) - t(1); %% เวลา

% Actuator speeds
VX = New_dx-dx / dt;
VY = New_dy-dy / dt;
VZ = New_dz-dz / dt;


% EE position from each leg
EE = zeros(3, length(t));
for k = 1:length(t)
    EE(:,k) = (PZ3(:,k) + PX3(:,k) + PY3(:,k)) / 3;
end

% EE Velocity for each axis
vEE_x = diff(EE(2,:)) / dt;
vEE_y = diff(EE(1,:)) / dt;
vEE_z = diff(EE(3,:)) / dt;

vEE_total = vecnorm(diff(EE,1,2)) / dt;

TauZ = zeros(4,300);
TauX = zeros(4,300);
TauY = zeros(4,300);
% Jacobian & static force
for k = 1:length(t)

Jz_full = jacobian_Z_full(L1,L2,L3,thetaZ1(k),thetaZ2(k),thetaZ3(k));
Jx_full = jacobian_X_full(L1,L2,L3,thetaX1(k),thetaX2(k),thetaX3(k));
Jy_full = jacobian_Y_full(L1,L2,L3,thetaY1(k),thetaY2(k),thetaY3(k));

TauZ(:,k) = Jz_full' * W;
TauX(:,k) = Jx_full' * W;
TauY(:,k) = Jy_full' * W;
end


%% ================= DISPLAY RESULTS (FULL REPORT WITH UNITS) =================
disp(' ');
disp('============= TRIPTERON REPORT ===============');
disp(' ');

%% ---------------------------------------------
% 1) ROBOT PARAMETERS
%% ---------------------------------------------
disp('===== ROBOT PARAMETERS =====');
disp(['L1 = ', num2str(L1), '   m   (Link 1 length)']);
disp(['L2 = ', num2str(L2), '   m   (Link 2 length)']);
disp(['L3 = ', num2str(L3), '   m   (End-effector link offset)']);

disp(' ');
disp('===== INPUT FORCE / WRENCH PARAMETERS =====');
disp(['Fx = ', num2str(Fx), '   N']);
disp(['Fy = ', num2str(Fy), '   N']);
disp(['Fz = ', num2str(Fz), '   N']);
disp(['Mx = ', num2str(Mx), '   N·m']);
disp(['My = ', num2str(My), '   N·m']);
disp(['Mz = ', num2str(Mz), '   N·m']);
disp(' ');

disp('===== INPUT ACTUATOR POSITION (START → END) =====');
disp(['DX = ', num2str(DX(1)), ' → ', num2str(DX(end)), '   m']);
disp(['DY = ', num2str(DY(1)), ' → ', num2str(DY(end)), '   m']);
disp(['DZ = ', num2str(DZ(1)), ' → ', num2str(DZ(end)), '   m']);

disp(' ');
disp('===== EE POSITION CHANGE (ACTUAL) =====');
disp(['EE X: ', num2str(dx), '  →  ', num2str(New_dx), '   m']);
disp(['EE Y: ', num2str(dy), '  →  ', num2str(New_dy), '   m']);
disp(['EE Z: ', num2str(dz), '  →  ', num2str(New_dz), '   m']);

%% ---------------------------------------------
% 2) FULL JACOBIAN MATRICES (6×4)
%% ---------------------------------------------

% disp(' ');
% disp('===== FULL JACOBIAN MATRICES (6×4) =====');
% 
% 
% disp('--- Z-leg Jacobian (Jv + Jw) ---');
% disp(Jz_full);
% 
% disp('--- X-leg Jacobian (Jv + Jw) ---');
% disp(Jx_full);
% 
% disp('--- Y-leg Jacobian (Jv + Jw) ---');
% disp(Jy_full);

%% ---------------------------------------------
% 3) VELOCITY RESULTS
%% ---------------------------------------------
disp(' ');
disp('===== VELOCITY RESULTS =====');
disp('--- EE VELOCITY (avg) ---');
disp(['v_EE_x = ', num2str(mean(abs(vEE_x))), '   m/s']);
disp(['v_EE_y = ', num2str(mean(abs(vEE_y))), '   m/s']);
disp(['v_EE_z = ', num2str(mean(abs(vEE_z))), '   m/s']);
disp(['v_EE_total = ', num2str(mean(abs(vEE_total))), '   m/s']);

disp(' ');
disp('--- ACTUATOR SPEED (avg) ---');
disp(['Actuator X = ', num2str(abs(VX)), '   m/s']);
disp(['Actuator Y = ', num2str(abs(VY)), '   m/s']);
disp(['Actuator Z = ', num2str(abs(VZ)), '   m/s']);

%% ---------------------------------------------
% 4) STATIC TORQUE FROM WRENCH
%% ---------------------------------------------
disp(' ');
disp('===== STATIC TORQUE (from W = [Fx Fy Fz Mx My Mz]) =====');
disp('Units: Newton-meter (N·m)');

%% Z-leg torque 
disp('--- Z-leg (Joint0 = P, Joint1-3 = R) ---');
disp([' τZ0 (P)  = ', num2str(TauZ(4,k)), '   N·m']);
disp([' τZ1 (R1) = ', num2str(TauZ(3,k)), '   N·m']);
disp([' τZ2 (R2) = ', num2str(TauZ(2,k)), '   N·m']);
disp([' τZ3 (R3) = ', num2str(TauZ(1,k)), '   N·m']);

%% X-leg torque
disp('--- X-leg (Joint0 = P, Joint1-3 = R) ---');
disp([' τX0 (P)  = ', num2str(TauX(4,k)), '   N·m']);
disp([' τX1 (R1) = ', num2str(TauX(3,k)), '   N·m']);
disp([' τX2 (R2) = ', num2str(TauX(2,k)), '   N·m']);
disp([' τX3 (R3) = ', num2str(TauX(1,k)), '   N·m']);

%% Y-leg torque
disp('--- Y-leg (Joint0 = P, Joint1-3 = R) ---');
disp([' τY0 (P)  = ', num2str(TauY(4,k)), '   N·m']);
disp([' τY1 (R1) = ', num2str(TauY(3,k)), '   N·m']);
disp([' τY2 (R2) = ', num2str(TauY(2,k)), '   N·m']);
disp([' τY3 (R3) = ', num2str(TauY(1,k)), '   N·m']);

disp(' ');
disp('===================================');
