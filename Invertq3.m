
function q3 = Invertq3(q1,q2,angle)
    angle_radian = deg2rad(angle);
    ck = angle_radian - q1 - q2 ;
    if ck < - 2*pi
        q3 = ck + 2*pi;
    elseif (ck > 2*pi)
        q3 = ck - 2*pi;
    else
        q3 = ck;
    end
end