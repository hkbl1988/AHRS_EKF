function g_n = glocal(L,h)

    wgs_84_parameters;


    g_n = zeros(3,1);
    g_0 = (9.7803253359/(sqrt(1 - (e*sin(L))^2)))*( 1 + 0.001931853*(sin(L))^2);
    k = 1 - (2*h/R_0)*(1 + f + ((omega_ie*R_0)^2)*(R_P/mu_E))...
            + 3*(h/R_0)^2;
    g_n(3,1) = k*g_0;  

end

