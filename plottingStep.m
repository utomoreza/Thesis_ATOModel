figure,
plot(Timej,[v v_ref]);
grid on;
axis([0 30 0 22]);
hold on

figure,
plot(Timej,[v v_ref]);
grid on;
axis([18.85 19 19.99 20.01])

pid parameters A:
best for step size of 1
kp = 1;
ki = 1e-2;
kd = 0;
************************************************
step size of 1:
Tr_Asize1 = 0.931 s
Ts_Asize1 (sse <3 percents) = 2.961 s;
Mp_Asize1 = 1.4653 m/s or 5.275 km/h
T_Mp_Asize1 = [1.36491017674337];
************************************************
step size of 20:
Tr_Asize20 = 18.784 s
Ts_Asize20 (sse <3 percents) = 46.862 s
Mp_Asize20 = 22.246 m/s or 80.0856 km/h
T_Mp_Asize20 = 20.9219535061770 s
% ************************************************
figure, % plotting step size of 1
plot(Timej,[v v_ref]);
grid on;
hold on;
axis([0 45 0 1.4])
Tr_Asize1 = 17.107; %0.9314899;
Ts_Asize1 = 18.15; %2.961;
Mp_Asize1 = max(v);
T_Mp_Asize1 = [17.3251279142964];%[1.36491017674337];
txt_Tr_Asize1_1 = ['\leftarrow Rise time = ',num2str(Tr_Asize1),' s '];
txt_Ts_Asize1_2 = ['\leftarrow Settling time = ',num2str(Ts_Asize1),' s'];
txt_Mp_Asize1_3 = ['\leftarrow Max overshoot = ',num2str(Mp_Asize1),' m/s or +',num2str(Mp_Asize1/1*100-100), ' %'];
text(Tr_Asize1,1,txt_Tr_Asize1_1)
text(Ts_Asize1,(1+1/100*3),txt_Ts_Asize1_2)
text(T_Mp_Asize1,Mp_Asize1,txt_Mp_Asize1_3)
title('Type A parameters - Step size of 1');
xlabel('Time (s)');
ylabel('Speed (m/s)');
% ************************************************
figure, % plotting step size of 20
plot(Timej,[v v_ref]);
grid on;
axis([0 250 0 24])
hold on;
Tr_Asize20 = 18.272;%18.784;
Ts_Asize20 = 47.456;%46.862;
Mp_Asize20 = max(v);
T_Mp_Asize20 = [22.6008308697024];%20.9219535061770;
txt_Tr_Asize20_1 = ['\leftarrow Rise time = ',num2str(Tr_Asize20),' s'];
txt_Ts_Asize20_2 = ['\rightarrow Settling time = ',num2str(Ts_Asize20),' s'];
txt_Mp_Asize20_3 = ['\leftarrow Max overshoot = ',num2str(Mp_Asize20),' m/s or +',num2str(Mp_Asize20/20*100-100), ' %'];
text(Tr_Asize20,20,txt_Tr_Asize20_1)
text(Ts_Asize20,(20+20/100*3),txt_Ts_Asize20_2)
text(T_Mp_Asize20,Mp_Asize20,txt_Mp_Asize20_3)
title('Type A parameters - Step size of 20');
xlabel('Time (s)');
ylabel('Speed (m/s)');


pid parameters B:
best for step size of 20
kp = 42;
ki = 1e-5;
kd = 25;
************************************************
step size of 1:
Tr_Bsize1 = 0.931 s
Ts_Bsize1 (sse <3 percents) = never reach SSE <3 percents
overshoot_Bsize1 (<5 percents) = never reach overshoot/undershoot <5 percents
Mp_Bsize1 =  1.7413 m/s or 6.2687 km/h;
T_Mp_Bsize1 = 1511.91974994640 s
************************************************
step size of 20:
Tr_Bsize20 = 18.880 s
Ts_Bsize20 (sse <3 percents) = 18.207 s
overshoot_Bsize20 (<5 percents)
Mp_Bsize20 = 20.0002 m/s or 72.0007 km/h
T_Mp_Bsize20 = 18.9403614639004 s
% ************************************************
figure, % plotting step size of 1
plot(Timej,[v v_ref]);
grid on;
%axis([0 30 0 2])
hold on;
Tr_Bsize1 = find(v>0 & v<1,1,'last');
Tr_Bsize1 = Timej(Tr_Bsize1,1);
%Ts = no settling time
Mp_Bsize1 = max(v);
T_Mp_Bsize1 = [1645.34134649112];%[1511.91974994640];
txt_Tr_Bsize1_1 = ['\leftarrow Rise time = ',num2str(Tr_Bsize1),' s'];
txt_Ts_Bsize1_2 = 'Never reach steady state error <3%';
txt_Mp_Bsize1_3 = ['\downarrow Max overshoot = ',num2str(Mp_Bsize1),' m/s or +',num2str(Mp_Bsize1/1*100-100), ' %'];
text(Tr_Bsize1,1,txt_Tr_Bsize1_1)
text(10,0.1,txt_Ts_Bsize1_2)
text(T_Mp_Bsize1,Mp_Bsize1,txt_Mp_Bsize1_3)
title('Type B parameters - Step size of 1');
xlabel('Time (s)');
ylabel('Speed (m/s)');
% ************************************************
figure, % plotting step size of 20
plot(Timej,[v v_ref]);
grid on;
axis([0 150 0 22])
hold on;
Tr_Bsize20 = find(v>19 & v<20,1,'last');
Tr_Bsize20 = Timej(Tr_Bsize20,1);
Ts_Bsize20 = 18.192;%18.54;%18.207;
Mp_Bsize20 = max(v);
T_Mp_Bsize20 = [19.0757802291147];%[255.619903806108];%18.9403614639004;
txt_Tr_Bsize20_1 = ['\leftarrow Rise time = ',num2str(Tr_Bsize20),' s'];
txt_Ts_Bsize20_2 = ['\leftarrow Settling time = ',num2str(Ts_Bsize20),' s'];
txt_Mp_Bsize20_3 = ['\downarrow Max overshoot ',num2str(Mp_Bsize20),' or +',num2str(Mp_Bsize20/20*100-100), ' %'];
text(Tr_Bsize20,20,txt_Tr_Bsize20_1)
text(Ts_Bsize20,(20-20/100*3),txt_Ts_Bsize20_2)
text(T_Mp_Bsize20,Mp_Bsize20+0.5,txt_Mp_Bsize20_3)
title('Step size of 20 - Improved KF');%('Type B parameters - Step size of 20');
xlabel('Time (s)');
ylabel('Speed (m/s)');