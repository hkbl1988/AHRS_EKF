function p_e = wgslla2xyz(lat,lon,alt)


    %%经纬度数据—>ecef坐标值


    wgs_84_parameters;


    R_E = R_0/(sqrt(1 - (e*sind(lat))^2));
    
    

    p_e(1,1) = (R_E + alt)*cosd(lat)*cosd(lon);
    p_e(2,1) = (R_E + alt)*cosd(lat)*sind(lon);
    p_e(3,1) = ((1 - e^2)*R_E + alt)*sind(lat);

end

