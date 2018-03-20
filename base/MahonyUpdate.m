function [att2,exyzOut] = MahonyUpdate(att1,gyro,acc,mag,ts,Ki,Kp,exyzInt)

    %%AHRS姿态更新算法
    %%gyro unit rad/s
    %%acc unit m/s^2
    
    nm = norm(acc);
    if nm>0
        acc = acc/nm;
    else
        acc = [0;0;0];
    end
    
    nm = norm(mag);
    if nm>0
        mag = mag/nm;
    else
        mag = [0,0,0];
    end
    
    cbn = att2cbn(att1);    %%姿态角转换为姿态矩阵
    q1 = att2quat(att1);    %%姿态角转换为四元数
    
    exyz = (cross(cbn(3,:)',acc'))';
    exyzOut = exyzInt + exyz*Ki*ts;
    ang = (gyro + Kp*exyz + exyzOut)*ts;
    q = quatmul(q1,ang2quat(ang'));   %%四元数更新
    att2 = quat2att(q);     %%计算姿态角
    
end