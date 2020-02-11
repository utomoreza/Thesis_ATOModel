load v_lim;
v_lim(1) = 0;
v_lim(end) = 0;
load v_ref
figure,
plot(v_lim);
grid on
title('Speed Limit')
xlabel('Distance (m)')
ylabel('Speed (m/s)')
axis([-50 5050 0 23])

v_lim(1) = 0;
v_lim(end) = 0;
figure,
plot(s,v,'b-')
hold on
plot(s,v_lim,'k--');
grid on
A = 'A'; B = 'B'; C = 'C'; D = 'D'; E = 'E';
text(50,1,A)
text(3600,21,B)
text(4300,12,C)
text(4750,11,D)
text(4800,1,E)
title('Train Trajectory with Coasting Strategy')
xlabel('Distance (m)')
ylabel('Speed (m/s)')
legend('Train trajectory','Speed limit')
axis([-50 5050 0 23])

figure,
plot(s,accel,'b--')
hold on
plot(s,accel_B,'r--')
plot(s,acceleration,'k-')
grid on
title('Combination of Acceleration & Deceleration')
xlabel('Distance (m)')
ylabel('Acceleration/Deceleration (m/s^2)')
legend('Acceleration','Negative deceleration','Combination')

alti = cumtrapz(s(:,1),grad(:,2))/100;
figure,plot(alti);
grid on
title('Altitude')
xlabel('Distance (m)')
ylabel('Height (m)')
axis([0 5000 -3 18])