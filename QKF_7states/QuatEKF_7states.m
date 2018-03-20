clear
clc

if(ispc)
    addpath ..\base;
else
    addpath ../base;
end 

load('..\data\sensor.mat');

d2r = pi/180;
r2d = 180/pi;
g = 9.78;

len = length(IMU(:,1));

att_ins_sys = zeros(len,3);
att_ins_ins = zeros(len,3);
quat_ins_sys = zeros(len,4);
quat_ins_ins = zeros(len,4);
x_est = zeros(len,7);


%%噪声信息输入
wn_var  = 1e-6 * ones(1,4);% rot vel var
wbn_var = 1e-8* ones(1,3);% gyro bias change var
an_var  = 1e-1 * ones(1,3);% acc var
mn_var  = 1e-1 * ones(1,3);% mag var
Q = diag([wn_var, wbn_var]); 
R = diag([an_var, mn_var]); 
P = eye(7);


%%初始化处理
attInit = IMU_Ref(1,1:3);

attInit = [attInit(1,1),attInit(1,2),attInit(1,3)];

quat_ins_sys(1,:) = att2quat(attInit*d2r);
att_ins_sys(1,:) = attInit;

quat_ins_ins(1,:) = quat_ins_sys(1,:);
att_ins_ins(1,:) = att_ins_sys(1,:);


x_est(1,1:4) = quat_ins_sys(1,:);

IMU(:,4:6) = -(IMU(:,4:6)/10)*g;

acceler = zeros(len,3);
mag = zeros(len,3);

z = zeros(len,6);
innov = zeros(len,6);
gbias = zeros(len,3);

for k = 2:len
    
    wx = (IMU(k,1)*d2r);
    wy = (IMU(k,2)*d2r);
    wz = (IMU(k,3)*d2r);
    
    quat_ins_ins(k,:) = quatmul(quat_ins_ins(k-1,:),ang2quat([wx,wy,wz]'*dt));
    
    acceler(k,:) = IMU(k,4:6)./norm(IMU(k,4:6));
    mag(k,:) = IMU(k,7:9)./norm(IMU(k,7:9));
    
    q0 = x_est(k-1,1); q1 = x_est(k-1,2); q2 = x_est(k-1,3); q3 = x_est(k-1,4);
    bwx = x_est(k-1,5); bwy = x_est(k-1,6); bwz = x_est(k-1,7);
    
    z(k,:) = [acceler(k,:),mag(k,:)];
    
    x_est(k,:) = [  q0 - (q1*(wx-bwx)*dt)/2 - (q2*(wy - bwy)*dt)/2 - (q3*(wz - bwz)*dt)/2;
                    q1 + (q0*(wx-bwx)*dt)/2 - (q3*(wy - bwy)*dt)/2 + (q2*(wz - bwz)*dt)/2;
                    q2 + (q3*(wx-bwx)*dt)/2 + (q0*(wy - bwy)*dt)/2 - (q1*(wz - bwz)*dt)/2;
                    q3 - (q2*(wx-bwx)*dt)/2 + (q1*(wy - bwy)*dt)/2 + (q0*(wz - bwz)*dt)/2;
                                                                                      bwx;
                                                                                      bwy;
                                                                                      bwz];
    x_est(k,1:4) = x_est(k,1:4)/norm(x_est(k,1:4));                                                                       
    
    F = [  0,       - wx/2,       - wy/2,       - wz/2,  q1/2,  q2/2,  q3/2;
        wx/2,            0,         wz/2,       - wy/2, -q0/2,  q3/2, -q2/2;
        wy/2,       - wz/2,            0,         wx/2, -q3/2, -q0/2,  q1/2;
        wz/2,         wy/2,       - wx/2,            0,  q2/2, -q1/2, -q0/2;
           0,            0,            0,            0,     0,     0,     0;
           0,            0,            0,            0,     0,     0,     0;
           0,            0,            0,            0,     0,     0,     0];
       
       
   q0 = x_est(k,1); q1 = x_est(k,2); q2 = x_est(k,3); q3 = x_est(k,4);
                  
   PHI = eye(7) + F * dt;         
    
    quat = x_est(k,1:4);
    P = PHI * P * PHI' + Q;
    cbn = quat2cbn(quat);

    mR = cbn*mag(k,:)';
    bx = norm([mR(1),mR(2)]);
    bz = mR(3);
    
    h = [cbn'*[0;0;-1];cbn'*[bx;0;bz]];%%计算预测值
    
    innov(k,:) = h - z(k,:)'; 
    
    H = calcH(bx,bz,q0,q1,q2,q3);

    Hk_1 = [  2*q2, -2*q3, 2*q0, -2*q1, 0, 0, 0;
             -2*q1, -2*q0,-2*q3, -2*q2, 0, 0, 0;
             -2*q0,  2*q1, 2*q2, -2*q3, 0, 0, 0];

    Hk_2 = 2*[ -2*bz*q2,               2*bz*q3,               -4*bx*q2-2*bz*q0,      -4*bx*q3+2*bz*q1 0 0 0;
               -2*bx*q3+2*bz*q1,	2*bx*q2+2*bz*q0,	2*bx*q1+2*bz*q3,       -2*bx*q0+2*bz*q2 0 0 0;
                2*bx*q2,                2*bx*q3-4*bz*q1,	2*bx*q0-4*bz*q2,        2*bx*q1  0 0 0];
  
    H = [Hk_1;Hk_2];

    S = H*P*H' + R;
    K = P*H'/S;

    x_est(k,:) = x_est(k,:) - (K*innov(k,:)')';      %%得到估计状态
    I = eye(length(P));
    P = (I - K*H)*P;            % 更新协方差矩阵
    

    x_est(k,1:4) = x_est(k,1:4)/norm(x_est(k,1:4));

    quat_ins_sys(k,:) = x_est(k,1:4);
    att_ins_sys(k,:) = quat2att(quat_ins_sys(k,:))*r2d;
    att_ins_sys(k,3) = att_ins_sys(k,3) - 8.3;
    
    att_ins_ins(k,:) = quat2att(quat_ins_ins(k,:))*r2d;
    
    gbias(k,:) = x_est(k,5:7);
    
end

plot_ekf_result;