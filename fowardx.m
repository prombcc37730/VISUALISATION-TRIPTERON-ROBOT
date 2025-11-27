function T = fowardx(a, d, alpha, theta)
     A = [1,0,0,0;
         0,1,0,0;
         0,0,1,a;
         0,0,0,1];
    Alpha = [cos(alpha),sin(alpha),0,0;
             -sin(alpha),cos(alpha),0,0;
             0,0,1,0;
             0,0,0,1];
    D = [1,0,0,d;
         0,1,0,0;
         0,0,1,0;
         0,0,0,1];
    Theta = [1,0,0,0;
             0,cos(theta),sin(theta),0;
             0,-sin(theta),cos(theta),0;
             0,0,0,1];

    T = A*Alpha*D*Theta;
end