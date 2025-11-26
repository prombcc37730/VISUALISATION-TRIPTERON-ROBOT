
function q1 = Invertq1(L1, L2, L3, px, py,q1,angle)

      angle_radian = deg2rad(angle);
      X = px - L3*cos(angle_radian);
      Y = py - L3*sin(angle_radian);
      k1 = L1 + L2*cos(q1);
      k2 = L2*sin(q1);
      r = sqrt((k1^2)+(k2^2));
      cosAE = X /r;
      sinAE = Y/ r;

      qAE = atan2(sinAE,cosAE);
      qE = atan2(k2,k1);
      q1 = qAE -qE;

end