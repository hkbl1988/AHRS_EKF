
%%误差方程的建立基于平台失准角

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

Freq = 50;
dt = 1/Freq;

len = length(IMU(:,1));

att_ins_sys = zeros(len,3);
att_ins_ins = zeros(len,3);
quat_ins_sys = zeros(len,4);
quat_ins_ins = zeros(len,4);
x_est = zeros(len,6);


%%噪声信息输入

wn_var  = 1e-5 * ones(1,3);               % rot vel var
wbn_var = 1e-9* ones(1,3);                % gyro bias change var
an_var  = 1e-3 * ones(1,3);               % acc var
mn_var  = 1e-4 * ones(1,3);               % mag var
Q = diag([wn_var, wbn_var]); 
R = diag([an_var, mn_var]); 
q_var_init = 1e-5 * ones(1,3);            % init rot var
wb_var_init = 1e-7 * ones(1,3);           % init gyro bias var
P = diag([q_var_init, wb_var_init]);



%%初始化处理
attInit = IMU_Ref(1,1:3);
quat_ins_sys(1,:) = att2quat(attInit*d2r);
att_ins_sys(1,:) = attInit;
quat_ins_ins(1,:) = quat_ins_sys(1,:);
att_ins_ins(1,:) = att_ins_sys(1,:);

IMU(:,4:6) = -(IMU(:,4:6)/10)*g;

acceler = zeros(len,3);
mag = zeros(len,3);

z = zeros(len,6);
innov = zeros(len,6);

gbias = zeros(len,3);

Pcov = zeros(len,6);
Pcov(1,:) = sqrt(diag(P(:,:)));

for k = 2:len
    
    omega = (0.5*(IMU(k,1:3) + IMU(k-1,1:3))*d2r) ;
    
    quat_ins_ins(k,:) = quatmul(quat_ins_ins(k-1,:),ang2quat(omega'*dt));
    quat_ins_sys(k,:) = quatmul(quat_ins_sys(k-1,:),ang2quat(omega'*dt));
    
    acceler(k,:) = IMU(k,4:6)./norm(IMU(k,4:6));
    mag(k,:) = IMU(k,7:9)./norm(IMU(k,7:9));
    
    q = quat_ins_sys(k,:);
    cbn = quat2cbn(q);
    
    %%===================建立状态方程================================%%
    PHI = zeros(6,6);
    PHI(1:3,1:3) = eye(3);
    PHI(1:3,4:6) = -cbn * dt;
    PHI(4:6,4:6) = eye(3);
    
    G = [eye(3),zeros(3,3);
         zeros(3,3),-cbn];
     
    Qk = G*Q*G'*dt;
    
    P = PHI * P * PHI' + Qk;        %%完成一步预测
    
    mR = cbn*mag(k,:)';
    bx = norm([mR(1),mR(2)]);
    bz = mR(3);
    
    %%==========================以下计算量测矩阵=============================%%

    H = calcHrotang(bx,bz,q(1),q(2),q(3),q(4));
               
    detZ = [acceler(k,:)' - cbn'*[0;0;-1] ; mag(k,:)' - cbn'*[bx,0,bz]'];

    %%=========================开始滤波计算=====================================%%
    K = (P*H') / ( H*P*H' + R); 
    
    x_est(k,:) =  K * detZ;
    I = eye(length(P));
    P = (I - K*H)*P;            % 更新协方差矩阵
    
    Pcov(k,:) = sqrt(diag(P(:,:)));
    
    %%=======================开始校正============================================%%
    Cbn_minus = cbn;
    Cbn_plus = (eye(3) + skew(x_est(k,1:3)))*Cbn_minus;    
    quat_ins_sys(k,:) = cbn2quat(Cbn_plus); %%更新系统四元数
    quat_ins_sys(k,:) = quat_ins_sys(k,:)/norm(quat_ins_sys(k,:));
    
    gbias(k,:) = gbias(k-1,:) + x_est(k,4:6);

    att_ins_sys(k,:) = quat2att(quat_ins_sys(k,:))*r2d;
    att_ins_sys(k,3) = att_ins_sys(k,3) - 8.3;
    att_ins_ins(k,:) = quat2att(quat_ins_ins(k,:))*r2d;
    att_ins_ins(k,3) = att_ins_ins(k,3) - 8.3;
    
end

plot_ekf_result;