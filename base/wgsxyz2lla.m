function [lat, lon, alt] = wgsxyz2lla(p_e)

    %%ecef坐标->经纬度坐标

    wgs_84_parameters;



    p_n = zeros(3,1);
    x = p_e(1);         
    y = p_e(2);
    z = p_e(3);


    lon = atan2(y,x)*(180/pi);  %%经度



    p = norm([x y]);
    E = sqrt(R_0^2 - R_P^2);
    F = 54*(R_P*z)^2;
    G = p^2 + (1 - e^2)*z^2 - (e*E)^2;
    c = e^4*F*p^2/G^3;
    s = (1 + c + sqrt(c^2 + 2*c))^(1/3);
    P = (F/(3*G^2))/((s + (1/s) + 1)^2);
    Q = sqrt(1 + 2*e^4*P);
    k_1 = -P*e^2*p/(1 + Q);
    k_2 = 0.5*R_0^2*(1 + 1/Q);
    k_3 = -P*(1 - e^2)*z^2/(Q*(1 + Q));
    k_4 = -0.5*P*p^2;
    r_0 = k_1 + sqrt(k_2 + k_3 + k_4);
    k_5 = (p - e^2*r_0);
    U = sqrt(k_5^2 + z^2);
    V = sqrt(k_5^2 + (1 - e^2)*z^2);

    alt = U*(1 - (R_P^2/(R_0*V)));      %%高度



    z_0 = (R_P^2*z)/(R_0*V);
    e_p = (R_0/R_P)*e;

    lat = atan((z + z_0*(e_p)^2)/p)*(180/pi);   %%纬度


end
