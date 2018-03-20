function att = quat2att(q)

    q11 = q(1)*q(1);
    q12 = q(1)*q(2);
    q13 = q(1)*q(3);
    q14 = q(1)*q(4);
    q22 = q(2)*q(2);
    q23 = q(2)*q(3);
    q24 = q(2)*q(4);
    q33 = q(3)*q(3);
    q34 = q(3)*q(4);
    q44 = q(4)*q(4);
    
    roll = atan2(2*(q12+q34),q11-q22-q33+q44);
    pitch = asin(2*(q13 - q24));
    yaw = atan2(2*(q23+q14),q11+q22-q33-q44);
    
    att = [roll,pitch,yaw];

end