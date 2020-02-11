% PLOTTING ACTUAL SPEED IN TRACTION PART WITH SETTLING TIME
sse_value = v_ref2(1,1)-v_ref2(1,1)/100*3;
T_set_idx = find(v>sse_value,1);
T_set = Timej(T_set_idx,1);

figure
plot(Timej, v,'b-');
hold on
plot(Timej, v_ref2,'k--');
plot(Timej(T_set_idx),v(T_set_idx),'o');
grid on
xlabel('Time (s)')
ylabel('Speed (m/s)')
% title('Actual speed without controller')
% title('Actual speed of PID')
% title('Actual speed of PD')
title('Actual speed of Fuzzy-PD')
legend('Actual speed','Reference')
axis([0 15 0 11]) % FOR WITHOUT CONTROLLER, PD, FUZZY-PD
% axis([0 300 0 15.5]) % FOR PID
txt_TS = ['\leftarrow Settling time ',num2str(T_set),' s'];
text(Timej(T_set_idx),v(T_set_idx),txt_TS)
% FOR APPOINTING SETTLING TIME OF UNSTABLE CONTROL RESPONSE
%     txt_TS = ['\leftarrow Settling time ',num2str(Timej(1646)),' s'];
%     text(Timej(1646),v(1646),txt_TS)
% *****************

% PLOTTING ACTUAL SPEED IN TRACTION PART
figure
plot(s, v,'b-');
hold on
plot(s, v_ref2,'k--');
grid on
xlabel('Distance (m)')
ylabel('Speed (m/s)')
% title('Actual speed without controller')
% title('Actual speed of PID')
% title('Actual speed of PD')
title('Actual speed of Fuzzy-PD')
legend('Actual speed','Reference')
axis([0 100 0 12]) % FOR WITHOUT CONTROLLER, PD, FUZZY-PD
% axis([0 1700 0 15.5]) % FOR PID

% PLOTTING ACC IN TRACTION PART
% figure,
% plot(s, acceleration)
% grid on
% xlabel('Distance (m)')
% ylabel('Acceleration (m/s^2)')
% title('Acceleration')
% axis([0 100 0 1.2]) % FOR WITHOUT CONTROLLER, PD, FUZZY-PD
% axis([0 1800 -1.2 1.2]) % FOR PID

%%
% *************************************************************************
% PLOTTING ACTUAL SPEED IN BRAKING PART WITH SETTLING TIME
% T_set_1 = v_ref2(1,1)-v_ref2(1,1)/100*3;
% T_set_idx_1 = find(v>T_set_1,1);

sse_value_2 = v_ref2(500,1)+v_ref2(500,1)/100*3;
T_set_idx_2 = find(v>v_ref2(500,1) & v<sse_value_2,1,'last');
T_set_2 = Timej(T_set_idx_2,1);

figure
plot(Timej, v,'b-');
hold on
plot(Timej, v_ref2,'k--');
% plot(Timej(T_set_idx_1),v(T_set_idx_1),'o');
plot(Timej(T_set_idx_2),v(T_set_idx_2),'o');
grid on
xlabel('Time (s)')
ylabel('Speed (m/s)')
% title('Actual speed without controller')
% title('Actual speed of PID')
% title('Actual speed of PD')
title('Actual speed of Fuzzy-PD')
legend('Actual speed','Reference')
axis([26.5 60 0 22]) % for not controller, PD, Fuzzy-PD
% axis([26.5 1100 0 22]) % for PID
% txt_TS_1 = ['\leftarrow Settling time ',num2str(T_set_1),' s'];
% text(Timej(T_set_idx_1),v(T_set_idx_1),txt_TS_1)
txt_TS_2 = '\leftarrow Settling time';
txt_TS_2a = [num2str(T_set_2),' s'];
text(Timej(T_set_idx_2),v(T_set_idx_2),txt_TS_2)
text(Timej(T_set_idx_2),4,txt_TS_2a)
% FOR APPOINTING SETTLING TIME OF UNSTABLE CONTROL RESPONSE
%     txt_TS_2 = '\leftarrow Settling time';
%     txt_TS_2a = [num2str(Timej(1420)),' s'];
%     text(Timej(1420),v(1420),txt_TS_2)
%     text(Timej(1420),4,txt_TS_2a)
% ***************************

% PLOTTING ACTUAL SPEED IN BRAKING PART
figure
plot(s, v,'b-');
hold on
plot(s, v_ref2,'k--');
grid on
xlabel('Distance (m)')
ylabel('Speed (m/s)')
% title('Actual speed without controller')
% title('Actual speed of PID')
% title('Actual speed of PD')
title('Actual speed of Fuzzy-PD')
legend('Actual speed','Reference')
axis([340 600 0 22]) % for no controller, PD, Fuzzy-PD
% axis([340 1500 0 22]) % for PID

% PLOTTING ACC IN BRAKING PART
% figure,
% plot(s, acceleration)
% grid on
% xlabel('Distance (m)')
% ylabel('Acceleration (m/s^2)')
% title('Acceleration')
% axis([340 600 -1 0.1]) % for no controller
% axis([340 1500 -1.2 1.2]) % for PID

%%
% *************************************************************************
% PLOTTING COMPARISON OF SIMPLE CONTROL,PID,PD,FUZZY PD - TRACTION PART
load v_simple % you've to manually generate act spd of simple control first
load v_PID % you've to manually generate act spd of PID first
load v_PD % you've to manually generate act spd of PD first

figure,
plot(s,v_ref_ori,'k-');
hold on;
plot(s,v_simple,'b-');
plot(s,v_PID,'b--');
plot(s,v_PD,'r-');
plot(s,v,'r--');
xlabel('Distance (m)');
ylabel('Speed (m/s)');
legend('Reference','Simple control','PID','PD','Fuzzy-PD');
title('Actual speed of simple control,PID,PD,Fuzzy-PD')
axis([0 1700 0 15])
grid on;

%%
% *************************************************************************
% PLOTTING COMPARISON OF SIMPLE CONTROL,PID,PD,FUZZY PD - BRAKING PART
load v_simple % you've to manually generate act spd of simple control first
load v_PID % you've to manually generate act spd of PID first
load v_PD % you've to manually generate act spd of PD first

figure,
plot(s,v_ref_ori,'k-');
hold on;
plot(s,v_simple,'b-');
plot(s,v_PID,'b--');
plot(s,v_PD,'r-');
plot(s,v,'r--');
xlabel('Distance (m)');
ylabel('Speed (m/s)');
legend('Reference','Simple control','PID','PD','Fuzzy-PD');
title('Actual speed of simple control,PID,PD,Fuzzy-PD')
axis([340 1470 0 22.2])
grid on;