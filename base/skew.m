function S =skew(v_in)

%%计算反对称矩阵函数

    S =[   0     -v_in(3)    v_in(2); 
         v_in(3)    0       -v_in(1); 
        -v_in(2)  v_in(1)      0];


end
 

