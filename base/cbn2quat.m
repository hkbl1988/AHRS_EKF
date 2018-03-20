function q = cbn2quat(cbn)

    tr = cbn(1,1) + cbn(2,2) + cbn(3,3);

	if(tr > 0)
	
		S = sqrt(tr + 1.0) * 2.0;
		q(1) = 0.25 * S;
		q(2) = (cbn(3,2) - cbn(2,3)) / S;
		q(3) = (cbn(1,3) - cbn(3,1)) / S;
		q(4) = (cbn(2,1) - cbn(1,2)) / S;
	
    elseif((cbn(1,1) > cbn(2,2)) && (cbn(1,1) > cbn(3,3)))
	
		S = sqrt(1.0 + cbn(1,1) - cbn(2,2) - cbn(3,3)) * 2.0;
        q(1) = (cbn(3,2) - cbn(2,3)) / S;
        q(2) = 0.25 * S;
        q(3) = (cbn(1,2) + cbn(2,1)) / S;
        q(4) = (cbn(1,3) + cbn(3,1)) / S;
	
    elseif(cbn(2,2) > cbn(3,3))
	
		S = sqrt(1.0 - cbn(1,1) + cbn(2,2) - cbn(3,3)) * 2.0;
        q(1) = (cbn(1,3) - cbn(3,1)) / S;
        q(2) = (cbn(1,2) + cbn(2,1)) / S;
        q(3) = 0.25 * S;
        q(4) = (cbn(2,3) + cbn(3,2)) / S;
	
	else
	
		S = sqrt(1.0 - cbn(1,1) - cbn(2,2) + cbn(3,3)) * 2.0;
        q(1) = (cbn(2,1) - cbn(1,2)) / S;
        q(2) = (cbn(1,3) + cbn(3,1)) / S;
        q(3) = (cbn(2,3) + cbn(3,2)) / S;
        q(4) = 0.25 * S;
    end
    
     n = sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);
    
    %%归一化处理
    q(1) = q(1)/n;
    q(2) = q(2)/n;
    q(3) = q(3)/n;
    q(4) = q(4)/n;
	

end