function att = cbn2att(cbn)

    roll = atan2(cbn(3,2),cbn(3,3));
    pitch = asin(-cbn(3,1));
    yaw = atan2(cbn(2,1),cbn(1,1));
    
    att = [roll,pitch,yaw];

end