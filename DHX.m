function T = DHX(a, d, alpha, theta)
    T =[ cos(alpha)   sin(alpha)*sin(theta)   sin(alpha)*cos(theta)    d*cos(alpha);
         0                 cos(theta)            -sin(theta)           a;
         -sin(alpha)  cos(alpha)*sin(theta)   cos(alpha)*cos(theta)    -d*sin(alpha);
         0                     0                       0               1];
end
