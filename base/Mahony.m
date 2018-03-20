function [QuatCur,eOut] = Mahony(Gyroscope, Accelerometer, Magnetometer, QuatPsd,eInt, tau)

    %%Mahony算法进行姿态的更新
    
    Kp = 1.0730*5;
    Ki = 0.2878*5;
   
    q = QuatPsd;
    
    if(norm(Accelerometer) == 0),return; end
    
    Accelerometer = Accelerometer / norm(Accelerometer);
    
    v = [2*(q(2)*q(4) - q(1)*q(3)) 2*(q(3)*q(4) + q(1)*q(2)) q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];
    
    if(norm(Magnetometer) ~= 0)     %%磁力计数据可用时可以融合航向
        
        Magnetometer = Magnetometer / norm(Magnetometer);
        
        h = quatmul(q, quatmul([0 Magnetometer] , quatconj(q)));
        
        b = [0 norm([h(2) h(3)]) 0 h(4)];
        
        w = [2*b(2)*(0.5 - q(3)^2 - q(4)^2) + 2*b(4)*(q(2)*q(4) - q(1)*q(3))
             2*b(2)*(q(2)*q(3) - q(1)*q(4)) + 2*b(4)*(q(1)*q(2) + q(3)*q(4))
             2*b(2)*(q(1)*q(3) + q(2)*q(4)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2)]; 
        
        e = cross(Accelerometer, v) + cross(Magnetometer, w);
        
        
    else if(norm(Magnetometer) == 0)    %%磁力计数据不可用时只用加表融合水平姿态
            
         e = cross(Accelerometer, v);
            
        end
    end
        
        eOut = eInt + e*tau;
        
        Gyroscope = Gyroscope + Kp * e + Ki * eOut;
        
        qDot = ang2quat(Gyroscope'*tau);
        
        QuatCur = quatmul(q,qDot);
        
        QuatCur = QuatCur/norm(QuatCur);    %%归一化处理

end