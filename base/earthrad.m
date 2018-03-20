function [R_N, R_E] = earthrad(L)


    wgs_84_parameters;


    k = sqrt(1 - (e*sin(L))^2);
    R_N = R_0*(1 - e^2)/k^3;
    R_E = R_0/k;

end
