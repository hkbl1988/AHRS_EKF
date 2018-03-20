function Q_k = discrete_process_noise(F,G,dt,Q)


%%矩阵离散化计算过程噪声矩阵函数

    [r,c] = size(F);
    Q_k = (eye(r) + dt*F)*(dt*G*Q*G');

 end
