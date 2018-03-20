clear
clc

if(ispc)
    addpath E:\EFY_GNSS_INS\base;
else
    addpath E:/EFY_GNSS_INS/base;
end 

q0 = sym('q0','real');      %%四元数
q1 = sym('q1','real');
q2 = sym('q2','real');
q3 = sym('q3','real');

bwx = sym('bwx','real');    %%陀螺零偏
bwy = sym('bwy','real');
bwz = sym('bwz','real');

rotErr1 = sym('rotErr1','real');    %%横滚角误差
rotErr2 = sym('rotErr2','real');    %%俯仰角误差
rotErr3 = sym('rotErr3','real');    %%航向角误差

wx = sym('wx','real');      %%
wy = sym('wy','real');
wz = sym('wz','real');

dt = sym('dt','real');

mx = sym('mx','real');
my = sym('my','real');
mz = sym('mz','real');

bx = sym('bx','real');
bz = sym('bz','real');

rotErr = [rotErr1,rotErr2,rotErr3]; %%姿态角误差
bw = [bwx,bwy,bwz];


quat = [q0,q1,q2,q3];       %%四元数
Quat = [quat];
deltQuat = [1;0.5*wx;0.5*wy;0.5*wz];    %%计算四元数增量
quatNew = quatmulsyms(quat,deltQuat);   %%四元数更新


rotErrNew = -(eye(3) + skew([wx,wy,wz]))*rotErr' - bw'; %%姿态角误差更新
bwnew = bw;

stateVector = [rotErr , bw];
stateVectorNew = [rotErrNew'  bwnew];

PHI = jacobian(stateVectorNew', stateVector);      %%得到状态转移矩阵
matlabFunction(PHI,'file','calcPHI.m');




%%计算量测矩阵
cbn = quat2cbn(quat);
mR = cbn * [mx;my;mz];
pred = [cbn'*[0;0;-1];cbn'*[bx;0;bz]]; %%预测值(建立与旋转矢量误差之间的关系)

Ham = jacobian(pred,quat);      %%建立与四元数之间的关系

%%建立四元数与旋转矢量误差之间的雅克比矩阵

quatErr = [1;0.5*rotErr1;0.5*rotErr2;0.5*rotErr3];

quatErrNewRot = quatmulsyms(quat,quatErr);

Hxrot = jacobian(quatErrNewRot ,stateVector);
Hrot = Ham*Hxrot;
matlabFunction(Hrot,'file','calcHrotErr.m');

quatErrNewang = quatmulsyms(quatErr,quat);
Hxang = jacobian(quatErrNewang ,stateVector);
Hang = Ham*Hxang;
matlabFunction(Hang,'file','calcHrotang.m');











