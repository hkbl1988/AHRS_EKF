figure(1);
subplot(311);plot(att_ins_sys(:,1),'LineWidth',1.5);hold on;plot(att_ins_ins(:,1),'r','LineWidth',1.5);plot(IMU_Ref(:,1),'m','LineWidth',1.5);legend('eskf simulator','pure nav','system');ylabel('roll');grid on;
subplot(312);plot(att_ins_sys(:,2),'LineWidth',1.5);hold on;plot(att_ins_ins(:,2),'r','LineWidth',1.5);plot(IMU_Ref(:,2),'m','LineWidth',1.5);grid on;ylabel('pitch');
subplot(313);plot(att_ins_sys(:,3),'LineWidth',1.5);hold on;plot(att_ins_ins(:,3),'r','LineWidth',1.5);plot(IMU_Ref(:,3),'m','LineWidth',1.5);grid on;ylabel('yaw');

figure(2);
subplot(311);plot(gbias(:,1)*r2d,'LineWidth',1.5);title('Gx Bias(deg/s)');grid on;
subplot(312);plot(gbias(:,2)*r2d,'LineWidth',1.5);title('Gy Bias(deg/s)');grid on;
subplot(313);plot(gbias(:,3)*r2d,'LineWidth',1.5);title('Gz Bias(deg/s)');grid on;

figure(3)
subplot(311);plot(att_ins_sys(:,1) - IMU_Ref(:,1),'LineWidth',1.5);title('roll error(deg)');grid on;
subplot(312);plot(att_ins_sys(:,2) - IMU_Ref(:,2),'LineWidth',1.5);title('pitch error(deg)');grid on;
subplot(313);plot(att_ins_sys(:,3) - IMU_Ref(:,3),'LineWidth',1.5);title('yaw error(deg)');grid on;