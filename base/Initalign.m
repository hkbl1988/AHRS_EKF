function att = Initalign(Accelermeter,Yaw)

%%³õÊ¼¶Ô×¼º¯Êý

    g = 9.81;

    att(1) = atan(Accelermeter(2)/Accelermeter(3)); %%ºá¹ö½Ç
    att(2) = asin(Accelermeter(1)/g);               %%¸©Ñö½Ç
    att(3) = Yaw;                                   %%º½Ïò½Ç

end