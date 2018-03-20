
function yaw = magb2yaw(att,Magb)

    %%根据磁力计的输出和水平姿态计算磁航向
    
    Roll = att(1);  %%横滚角
    Pitch =att(2);  %%俯仰角
    
    Cb2 = [ cos(Pitch)    sin(Pitch)*sin(Roll)    sin(Pitch)*cos(Roll);
                0              cos(Roll)                 -sin(Roll);
           -sin(Pitch)    cos(Pitch)*sin(Roll)    cos(Pitch)*cos(Roll)];
       
       
     Magn = Cb2*Magb;   
     
     yaw = atan2(-Magn(2), Magn(1));


end