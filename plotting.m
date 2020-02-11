% % plotting speed
%load v_ref_ori
% figure
% plot(s, v.*3.6,'b-');
% hold on
% plot(s, v_ref2.*3.6,'g-.');
% plot(s, v_lim.*3.6,'k--');
% plot(s,v_noise.*3.6,'r:');
% grid on
% additional plotting
%load v_ref
%plot(s, v_ref.*3.6);
% ***************

% title('Speed comparison');
% xlabel('Distance (m)');
% ylabel('Speed (km/h)');
% legend('Actual speed', 'Speed reference','Speed limit','Noisy speed');

% plotting error deviation of speed profile against
% speed reference
% dev = v_ref2 - v;
% e_percent_pos = v_ref2./100*3;
% over_pos = v_ref2./100*5;
% e_percent_neg = -e_percent_pos;
% over_neg = -over_pos;
% figure,
% plot(s,dev);
% hold on;
% plot(s,e_percent_pos);
% plot(s,e_percent_neg);
% % plot(s,over_pos)
% % plot(s,over_neg)
% grid on;
% xlabel('Distance (m)')
% ylabel('Speed (m/s)')
% title('Error deviation between Vref and Vact')
% legend('error deviation','+3% error limit','-3% error limit')
% axis tight
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % plotting acceleration
% figure
% plot(s, accel)
% grid on
% title('Acceleration');
% xlabel('Distance (m)');
% ylabel('Acceleration (m/s^2)');
% 
% % plotting jerk
% figure
% plot(s, jerk);
% grid on
% title('Jerk');
% xlabel('Distance (m)');
% ylabel('Jerk (m/s^3)');

% plotting running time of actual speed
figure
plot(Timej./60,s);
hold on
plot(Time_ref./60,s);
grid on
title('Running time');
xlabel('Time (min)');
ylabel('Distance (m)');
legend('v_a_c_t','v_r_e_f')
axis tight

% plotting running time of ref speed
% figure
% plot(Time_ref./60,s);
% grid on
% title('Running time of speed reference');
% xlabel('Time (min)');
% ylabel('Distance (m)');

% Displaying running time difference
maxTimej = max(Timej);
DispMaxTime = ['Maximum time of actual speed is ',num2str(maxTimej), ...
    ' seconds or ', num2str(maxTimej/60), ' minutes.'];
disp(DispMaxTime);
maxTime_ref = max(Time_ref);
DispMaxTimeRef = ['Maximum time of speed reference is ',num2str(maxTime_ref), ...
    ' seconds or ', num2str(maxTime_ref/60), ' minutes.'];
disp(DispMaxTimeRef);
fiveseconds = maxTimej - maxTime_ref;
if fiveseconds >= 0 && fiveseconds <= 30
%     comparison = percentage - 100;
    DispTimeDif = ['\nTime difference between actual speed and ', ...
	'speed reference is that\n', 'actual speed is ', ...
	num2str(fiveseconds),' seconds later than reference speed.\n', ...
	'Therefore, time constraint is SATISFIED.'];
%     or\n+', num2str(comparison), ' per cent later.'];
    disp(sprintf(DispTimeDif));
elseif fiveseconds >= -30 && fiveseconds < 0
    DispTimeDif = ['\nTime difference between actual speed and ', ...
	'speed reference is that\n', 'actual speed is ', ...
	num2str(fiveseconds),' seconds earlier than reference speed.\n', ...
	'Therefore, time constraint is SATISFIED.'];
    disp(sprintf(DispTimeDif));
elseif fiveseconds > 30
    DispTimeDif = ['\nTime difference between actual speed and ', ...
        'speed reference is that\n', 'actual speed is ', ...
        num2str(fiveseconds),' seconds later than reference speed,\n', ...
        'ie >30 seconds. Therefore, time constraint is UNSATISFIED.'];
    disp(sprintf(DispTimeDif));
elseif fiveseconds < 30
%     comparison = -(100 - percentage);
    DispTimeDif = ['\nTime difference between actual speed and ', ...
        'speed reference is that\n', 'actual speed is ', ...
        num2str(fiveseconds),' seconds earlier than reference speed.\n', ...
        'ie <0 second. Therefore, time constraint is UNSATISFIED.'];
%     or\n', num2str(comparison), ' per cent earlier.'];
    disp(sprintf(DispTimeDif));
% else
% %     comparison = 0;
%     DispTimeDif = ['Actual speed is exactly the same as, ie', ...
%         num2str(maxTimej),'\nTherefore, time constraint is SATISFIED.'];
%     disp(sprintf(DispTimeDif));
end

exceederror_pos = zeros(size(s));
exceederror_neg = zeros(size(s));
for i = 1:length(dev)
    if dev(i,1) >= e_percent_pos(i,1)
        exceederror_pos(i,1) = 1;
    elseif dev(i,1) <= e_percent_neg(i,1)
        exceederror_neg(i,1) = 1;
    else
        exceederror_pos(i,1) = 0;
        exceederror_neg(i,1) = 0;
    end
end
total_passing_pos = sum(sum(exceederror_pos(2:end-1,1)));
total_passing_neg = sum(sum(exceederror_neg));
if total_passing_pos == 0 && total_passing_neg == 0
    satisfy = 'SATISFIED';
else
    satisfy = 'UNSATISFIED';
end
DispErrorDif = ['\nThe system ',num2str(total_passing_pos), ...
    ' times exceeds +5 percent error limit,\n', ...
    'and ',num2str(total_passing_neg),' times exceeds', ...
    ' -5 percent error limit.\n','Therefore, error constraint is ', ...
    num2str(satisfy),'.'];
disp(sprintf(DispErrorDif));

% CALCULATE IAE AND ISE
IAE_buffer = abs(error);
ISE_buffer = error.^2;
for n = 1:length(error)
    IAE_buffer(n,1) = IAE_buffer(n,1)*del_Tj(n,1);
    ISE_buffer(n,1) = ISE_buffer(n,1)*del_Tj(n,1);
end
IAE = sum(IAE_buffer);
ISE = sum(ISE_buffer);
DispIAE = ['\nIAE value is ',num2str(IAE),'.'];
disp(sprintf(DispIAE));
DispISE = ['ISE value is ',num2str(ISE),'.'];
disp(sprintf(DispISE));

% Calculate IAE
% IAE_buffer = abs(error);
% IAE = trapz(IAE_buffer);
% DispIAE = ['\nIAE value is ',num2str(IAE),'.'];
% disp(sprintf(DispIAE));
% 313.8141 > normal
% 313.0867 > absolute delta error
% 307.6119 > absolute delta & sigma error

% Calculate ISE
% ISE_buffer = error.^2;
% ISE = trapz(ISE_buffer);
% DispISE = ['ISE value is ',num2str(ISE),'.'];
% disp(sprintf(DispISE));
% 92.4331 > normal
% 91.9058 > absolute delta error
% 88.4099 > absolute delta & sigma error

% Calculate ITAE
% ITAE_buffer = Timej.*IAE_buffer;
% ITAE = trapz(ITAE_buffer)
% 1.1248e+05 > normal
% 1.1265e+05 > absolute delta error
% 1.1219e+05 > absolute delta & sigma error

% Calculate ITSE
% ITSE_buffer = Timej.*ISE_buffer;
% ITSE = trapz(ITSE_buffer)
% 2.0371e+04 > normal
% 2.0348e+04 > absolute delta error
% 1.9728e+04 > absolute delta & sigma error

% figure, % plotting running time using pid parameters type A/B
% plot(s,[Timej Time_ref]);
% grid on;
% axis([0 6500 0 350])
% title('Running time comparison');
% xlabel('Distance (m)');
% ylabel('Time (s)');
% legend('Actual speed','Speed reference');
% hold on;
% txt_ARunTime_1 = {'Max time','Actual speed',num2str(maxTimej)};
% txt_ARunTime_2 = {'Max time','Speed reference',num2str(maxTime_ref)};
% text(3800,maxTimej,txt_ARunTime_1)
% text(s(end,1),maxTime_ref,txt_ARunTime_2)

% figure, % plotting running time using pid parameters type B
% plot(s,[Timej Time_ref]);
% grid on;
% axis([0 6500 0 350])
% title('Running time comparison - Type B parameters)');
% xlabel('Distance (m)');
% ylabel('Time (s)');
% legend('Actual speed','Speed reference');
% hold on;
% txt_ARunTime_1 = {'Max time','Actual speed',num2str(maxTimej)};
% txt_ARunTime_2 = {'Max time','Speed reference',num2str(maxTime_ref)};
% text(s(end,1),maxTimej,txt_ARunTime_1)
% text(3800,maxTime_ref,txt_ARunTime_2)

% + 3.2128
% 1% late time is 324.4948

% +- 9.5421
% 3% late 330.9205

% PLOTTING RUNNING RESISTANCE
% FOR DISPLAY PURPOSE ONLY
% davis = [0.5 23 500];
% kec = 1:20;
% kec = kec';
% for i = 1:length(kec)
%     Rr_davisC(i,1) = davis(1).*kec(i,1)^2;
%     Rr_davisB(i,1) = davis(2).*kec(i,1);
%     Rr_davisA(i,1) = davis(3);
%     Rr(i,1) = Rr_davisC(i,1) + Rr_davisB(i,1) + Rr_davisA(i,1);
% end
% figure,
% plot(kec,Rr_davisA,'k:')
% hold on
% plot(kec,Rr_davisB,'k-.')
% plot(kec,Rr_davisC,'k--')
% plot(kec,Rr,'k-')
% yticklabels({})
% title('Running Resistance')
% xlabel('Speed (m/s)')
% ylabel('Resistive force (N)')
% legend('A','Bv','Cv^2','R_T')

% PLOTTING TRACTION VS SPEED
% FOR DISPLAY PURPOSE ONLY
% kec = 0:0.1:max_speed;
% kec = kec';
% F = zeros(size(kec));
% F = Power./kec;
% Res=zeros(size(kec));
% for i = 1:length(kec)
%     if kec(i,1) <= 10
%         F(i,1) = max_trac;
%     end
%     if kec(i,1) > 10 && kec(i,1) <= max_speed
%         F(i,1) = max_trac - (1000*(kec(i,1)-10));
%     end
% %     if trac(1,i)>max_accel(1)
% %         trac(1,i)=max_accel(1);
% %     end
%     Res(1,i)=Davis(1)+ kec(1, i)*Davis(2) + (kec(1, i))^2*Davis(3);
% end
% figure,plot(kec,[F Res])
% grid on

% Res=Res./inertial_mass;
% accel=trac-Res;

% PLOTTING ALTITUDE
%integral of the gradient profile:
% % grad_1 = zeros(size(s));
% % pos = 2;
% % for i = 1:fix(1000/del_S*max(gradient(:,1)))
% %    grad_1(1,i)=gradient(pos-1,2);
% %     if i*del_S>=fix(gradient(pos,1)*1000)
% %         pos=pos+1;
% %     end
% % end
% altitude = cumtrapz(s(:), grad(:));
% % altitude = cumsum(grad(:));
% figure,plot(s,altitude, 'k')
% title('Altitude - Stratford Int''l to Woolwich Arsenal')
% xlabel('Distance (m)')
% ylabel('Altitude (m)')
% grid on
% hold on
% for x = 1:length(station)
%     plot(station(x,2),altitude((station(x,2))+1),'bo')
% end
% legend('Altitude','Station position')
% h = text(station(1,2)+130,-16,station_name(1,:));
% set(h,'Rotation',90);
% h = text(station(2,2)-75,altitude((station(2,2)))-7.5,station_name(2,:));
% set(h,'Rotation',90);
% h = text(station(3,2),altitude((station(3,2)))-15,station_name(3,:));
% set(h,'Rotation',90);
% h = text(station(4,2),altitude((station(4,2)))-10,station_name(4,:));
% set(h,'Rotation',90);
% h = text(station(5,2),altitude((station(5,2)))-8,station_name(5,:));
% set(h,'Rotation',90);
% h = text(station(6,2),altitude((station(6,2)))-8,station_name(6,:));
% set(h,'Rotation',90);
% h = text(station(7,2),altitude((station(7,2)))-11,station_name(7,:));
% set(h,'Rotation',90);
% h = text(station(8,2),altitude((station(8,2)))-12,station_name(8,:));
% set(h,'Rotation',90);
% h = text(station(9,2),altitude((station(9,2)))-13,station_name(9,:));
% set(h,'Rotation',90);
% h = text(station(10,2)-150,altitude((station(10,2)))-14,station_name(10,:));
% set(h,'Rotation',90);
% h = text(station(11,2),altitude((station(11,2)))+1,station_name(11,:));
% set(h,'Rotation',90);
% h = text(station(12,2),altitude((station(12,2)))+0.7,station_name(12,:));
% set(h,'Rotation',90);
% thames = {'River','Thames'};
% text(9500,-11,thames,'Color','black','FontSize',12)

% PLOTTING PDF - NORMAL DISTRIBUTION (EXAMPLE)
% data1 = randn(100000,1);
% figure,
% histogram(data1,'Normalization','pdf')
% xticks([-4 -3 -2 -1 0 1 2 3 4]);
% xticklabels({' ',' ',' ',' ','\mu',' ',' ',' ',' '});
% set(gca,'YTickLabel',{' '})
% xlabel('Values - x')
% ylabel('pdf - f(x)')
% title('Probability Density Function - Normal Distribution')


% PLOTTING CONTROL AND OUTPUT SIGNALS
% OF USING RELAY-BASED AUTOTUNING
% figure,
% yyaxis left
% plot(Time,u)
% grid on
% title('Time versus control & output signals')
% xlabel('Time (s)')
% ylabel('Control signal')
% axis([10 60 -1.2 1.3])
% yyaxis right
% plot(Time,v)
% ylabel('Speed (m/s)')
% axis([10 60 0 1.8])
% legend('Control signal (u)','Speed (v)')
% ****************************************************