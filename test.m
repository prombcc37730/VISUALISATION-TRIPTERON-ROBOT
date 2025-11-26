clear; clc; close all;

workspace = [0,2*(sqrt(2))];

% Robot link lengths
L1 = 2*(sqrt(2));
L2 = 2*(sqrt(2));
L3 = 0.5;
dx = 0.5;
dy = 0.5;
dz = 0.5;
New_dx = 3;
New_dy = 3;
New_dz = 3;
% center = [dx,dy,dz];
% new_center = [New_dx,New_dy,New_dz];
% Time vector

t = linspace(0, 6, 300);
DZ = ((New_dz -dz)*t/6)+dz;
DX = ((New_dx -dx)*t/6)+dx;
DY = ((New_dy -dy)*t/6)+dy;


for i = 1:length(t)

    q2z(i) = Invertq2(L1,L2,L3,DX(i),DY(i),90);
    q1z(i) = Invertq1(L1,L2,L3,DX(i),DY(i),q2z(i) ,90);
    q3z(i)  = Invertq3(q1z(i) ,q2z(i) ,90);
    thetaZ1 = q1z;
    thetaZ2 = q2z;
    thetaZ3 = q3z;
    
    q2x(i) = Invertq2(L1,L2,L3,-(5.6569-DY(i)),DZ(i),180);
    q1x(i) = Invertq1(L1,L2,L3,-(5.6569-DY(i)),DZ(i),q2x(i),180);
    q3x(i) = Invertq3(q1x(i),q2x(i),180);
    thetaX1 = q1x;
    thetaX2 = q2x;
    thetaX3 = q3x;

    q2y(i) = -Invertq2(L1,L2,L3,DX(i),DZ(i),0);
    q1y(i) = Invertq1(L1,L2,L3,DX(i),DZ(i),q2y(i),0);
    q3y(i) = Invertq3(q1y(i),q2y(i),0);
    thetaY1 = q1y;
    thetaY2 = q2y;
    thetaY3 = q3y;
end


% Setup figure
figure('Color','white');
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('tripteron Robot Simulation');

xlim([-2*(sqrt(2)) 6]); ylim([-2*(sqrt(2)) 6+2*(sqrt(2))]); zlim([-2*(sqrt(2)) 6]);

view(100,45);   % 3D camera angle

trace = animatedline('Color','m','LineWidth',1.5);

for k = 1:length(t)

    %DH parameter
    T0Z = DHZ(0, DZ(k),0, 0);
    T01Z = T0Z * DHZ(0, 0,0, thetaZ1(k) );          % Joint 1 rotates Z
    T02Z = T01Z * DHZ(L1, 0,0, thetaZ2(k) );  
    T03Z = T02Z * DHZ(L2, 0, 0, thetaZ3(k) ); 
    T0EZ = T03Z * DHZ(L3, 0, 0, 0); 
   
    T0X = DHX(0, DX(k),0, 0);
    T01X = T0X * DHX(0, 0,0, thetaX1(k));          % Joint 1 rotates X
    T02X = T01X * DHX(L1, 0,0, thetaX2(k));  
    T03X = T02X * DHX(L2, 0, 0, thetaX3(k)); 
    T0EX = T03X * DHX(L3, 0, 0, 0); 

    T0Y = DHY(0, DY(k),0, 0);
    T01Y = T0Y * DHY(0, 0,0, thetaY1(k));          % Joint 1 rotates X
    T02Y = T01Y * DHY(L1, 0,0, thetaY2(k));  
    T03Y = T02Y * DHY(L2, 0, 0, thetaY3(k)); 
    T0EY = T03Y * DHY(L3, 0, 0, 0);
    
    %joint position
    pZ1 = [0;0;DZ(k)];
    pZ2 = T02Z(1:3,4);
    pZ3 = T03Z(1:3,4);
    pZ4 = T0EZ(1:3,4);
    
    pX1 = [DX(k);0;0];
    pX2 = T02X(1:3,4);
    pX3 = T03X(1:3,4);
    pX4 = T0EX(1:3,4);
    pX1(2)=pX1(2)+5.6569;
    pX2(2)=pX2(2)+5.6569;
    pX3(2)=pX3(2)+5.6569;
    pX4(2)=pX4(2)+5.6569;

    pY1 = [0;DY(k);0];
    pY2 = T02Y(1:3,4);
    pY3 = T03Y(1:3,4);
    pY4 = T0EY(1:3,4);

     cla;
     % Draw robot links (thick 3D lines)
    plot3([0 0], [0 0], [0 5.6569], 'k', 'LineWidth',2);
    plot3([0 0], [0 5.6569], [0 0], 'k', 'LineWidth',2);
    plot3([0 5.6569], [5.6569 5.6569], [0 0], 'k', 'LineWidth',2);

    plot3([pZ1(1) pZ2(1)], [pZ1(2) pZ2(2)], [pZ1(3) pZ2(3)], 'B', 'LineWidth',2);
    plot3([pZ2(1) pZ3(1)], [pZ2(2) pZ3(2)], [pZ2(3) pZ3(3)], 'b', 'LineWidth',2);
    plot3([pZ3(1) pZ4(1)], [pZ3(2) pZ4(2)], [pZ3(3) pZ4(3)], 'y', 'LineWidth',4);
    
    plot3([pX1(1) pX2(1)], [pX1(2) pX2(2)], [pX1(3) pX2(3)], 'r', 'LineWidth',2);
    plot3([pX2(1) pX3(1)], [pX2(2) pX3(2)], [pX2(3) pX3(3)], 'r', 'LineWidth',2);
    plot3([pX3(1) pX4(1)], [pX3(2) pX4(2)], [pX3(3) pX4(3)], 'y', 'LineWidth',4);

    plot3([pY1(1) pY2(1)], [pY1(2) pY2(2)], [pY1(3) pY2(3)], 'g', 'LineWidth',2);
    plot3([pY2(1) pY3(1)], [pY2(2) pY3(2)], [pY2(3) pY3(3)], 'g', 'LineWidth',2);
    plot3([pY3(1) pY4(1)], [pY3(2) pY4(2)], [pY3(3) pY4(3)], 'y', 'LineWidth',4);

    % Draw joints as spheres
    scatter3(pZ1(1), pZ1(2), pZ1(3), 80, 'filled','b');
    scatter3(pZ2(1), pZ2(2), pZ2(3), 80, 'filled','b');
    scatter3(pZ3(1), pZ3(2), pZ3(3), 80, 'filled','b');
    scatter3(pZ4(1), pZ4(2), pZ4(3), 120, 'filled', 'y');
    scatter3(pX1(1), pX1(2), pX1(3), 80, 'filled','r');
    scatter3(pX2(1), pX2(2), pX2(3), 80, 'filled','r');
    scatter3(pX3(1), pX3(2), pX3(3), 80, 'filled','r');
    scatter3(pX4(1), pX4(2), pX4(3), 120, 'filled', 'y');
    scatter3(pY1(1), pY1(2), pY1(3), 80, 'filled','g');
    scatter3(pY2(1), pY2(2), pY2(3), 80, 'filled','g');
    scatter3(pY3(1), pY3(2), pY3(3), 80, 'filled','g');
    scatter3(pY4(1), pY4(2), pY4(3), 120, 'filled', 'y');
    drawnow;
end
