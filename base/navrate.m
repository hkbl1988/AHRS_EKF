function nav_rate = navrate(v_n,p_n)

    [R_N,R_E] = earthrad(p_n(1));

    nav_rate(1,1) = v_n(2)/(R_E + p_n(3));
    nav_rate(2,1) = -v_n(1)/(R_N + p_n(3));
    nav_rate(3,1) = -v_n(2)*tan(p_n(1))/(R_E + p_n(3));

end

