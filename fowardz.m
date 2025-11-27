function T = fowardz(a, d, alpha, theta)
     A = [1,0,0,a;
         0,1,0,0;
         0,0,1,0;
         0,0,0,1];
    Alpha = [1,0,-sin(alpha),0;
             0,cos(alpha),-sin(alpha),0;
             0,sin(alpha),cos(alpha),0;
             0,0,0,1];
    D = [1,0,0,0;
         0,1,0,0;
         0,0,1,d;
         0,0,0,1];
    Theta = [cos(theta),sin(theta),0,0;
             -sin(theta),cos(theta),0,0;
             0,0,1,0;
             0,0,0,1];

    T = A*Alpha*D*Theta;
end


