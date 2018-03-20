function enu = wgsxyz2enu(p_e,ref_lat,ref_lon,ref_alt)
    
    %%经纬度->当地ENU直角坐标值

    p_e_ref = wgslla2xyz(ref_lat, ref_lon, ref_alt);
    delta_xyz = p_e - p_e_ref;



    enu(1,1)= -sind(ref_lon)*delta_xyz(1) + cosd(ref_lon)*delta_xyz(2);

    enu(2,1)= -sind(ref_lat)*cosd(ref_lon)*delta_xyz(1) - ...
                sind(ref_lat)*sind(ref_lon)*delta_xyz(2) + ...
                cosd(ref_lat)*delta_xyz(3);

    enu(3,1)= cosd(ref_lat)*cosd(ref_lon)*delta_xyz(1) + ...
                cosd(ref_lat)*sind(ref_lon)*delta_xyz(2) + ... 
                sind(ref_lat)*delta_xyz(3);      

            
end
