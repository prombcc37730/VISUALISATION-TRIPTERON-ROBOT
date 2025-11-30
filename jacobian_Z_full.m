function J = jacobian_Z_full(L1,L2,L3,q1,q2,q3)

S1   = sin(q1);     C1   = cos(q1);
S12  = sin(q1+q2);  C12  = cos(q1+q2);
S123 = sin(q1+q2+q3);
C123 = cos(q1+q2+q3);

J = [
    -L1*S1 - L2*S12 - L3*S123 ,  -L2*S12 - L3*S123 ,  -L3*S123 ,  0 ;
    -L1*C1 - L2*C12 - L3*C123 ,  -L2*C12 - L3*C123 ,  -L3*C123 ,  0 ;
     0                         ,   0                ,   0       ,  1 ;
];

end
