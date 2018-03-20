function ned = wgsxyz2ned(p_e,ref_lat,ref_lon,ref_alt)

    %%经纬度->当地NED坐标值

    enu = wgsxyz2enu(p_e,ref_lat,ref_lon,ref_alt);


    C = [0 1 0;1 0 0;0 0 -1];


    ned = C*enu;

end
