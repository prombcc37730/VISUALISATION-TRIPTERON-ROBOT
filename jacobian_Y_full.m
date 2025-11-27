function J = jacobian_Y_full(L1,L2,L3,q1,q2,q3)

S1   = sin(q1);     C1   = cos(q1);
S12  = sin(q1+q2);  C12  = cos(q1+q2);
S123 = sin(q1+q2+q3);  
C123 = cos(q1+q2+q3);

J = [
    -L3*S123  - L2*S12 - L1*S1 ,  - L3*S123 - L2*S12 ,  -L3*S123 ,   0 ;
    0 , 0 , 0 , 1 ;
   L3*C123 + L2*C12 + L1*C1 ,  L3*C123 + L2*C12 ,  L3*C123 ,   0 ;
    0 , 0 , 0 , 0 ;
    1 , 1 , 1 , 0 ;
    0 , 0 , 0 , 0 ;
];
end
