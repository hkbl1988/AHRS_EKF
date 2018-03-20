function q = att2quat(att)
    
    roll = att(1);
    pitch = att(2);
    yaw  = att(3);
    
    cosR = cos(roll/2.0);
    sinR = sin(roll/2.0);
    cosP = cos(pitch/2.0);
    sinP = sin(pitch/2.0);
    cosH = cos(yaw/2.0);
    sinH = sin(yaw/2.0);
    
    q(1) = cosH*cosP*cosR + sinH*sinP*sinR;
    q(2) = cosH*cosP*sinR - sinH*sinP*cosR;
    q(3) = cosH*sinP*cosR + sinH*cosP*sinR;
    q(4) = sinH*cosP*cosR - cosH*sinP*sinR;


end