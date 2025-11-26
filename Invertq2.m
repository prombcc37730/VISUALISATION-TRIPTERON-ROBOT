
function q2 = Invertq2(L1, L2, L3, px, py,angle)
      angle_radian = deg2rad(angle);
      X = px - L3*cos(angle_radian);
      Y = py - L3*sin(angle_radian);
      cosB = ((X^2)+(Y^2)-(L1^2)-(L2^2))/(2*L1*L2);
      
      sinB = sqrt(1 - cosB^2);
      % sinB2 = -sqrt(1 - cosB^2);

      q2 = atan2(sinB , cosB);
      % r_2 = atan2(sinB2 , cosB);

      % if r_1 > 1.5708 || r_1 < -1.5708
      %     q2 = r_2;
      % else if r_2 > 1.5708 || r_2 < -1.5708
      %      q2 = r_2  ; 
      % end     
    
end