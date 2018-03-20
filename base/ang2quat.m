function q = ang2quat(ang)

    %%旋转矢量转换为四元数
    
    [m,n] = size(ang);
    
    if m==3 
        norm = sqrt(ang'*ang);
    end
    if n==3
        norm = sqrt(ang*ang');
    end
    
    if norm > 1e-40
        f = sin(norm/2)/norm;
    else 
        f = 1/2;
    end
    
    q = [cos(norm/2);f*ang];
   
end
