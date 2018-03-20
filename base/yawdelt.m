function e = yawdelt(yaw , yaw_ref)
    
    if(length(yaw)~=length(yaw_ref)) 
        return
    else
        e = yaw - yaw_ref;
        
        for k  = 1:length(yaw)
             if(e(k) > 90)
                e(k) = e(k) - 360;
              elseif(e(k) < -90)
                e(k) = e(k) + 360;
              end 
        end  
    end
end