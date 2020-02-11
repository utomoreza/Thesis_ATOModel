% % Adding 5-second dwelling delay
k = del_S;
del_T(k+1,1) = 2*del_S/(v(k+1,1)+v(k,1)) + dwell;
Time(k+1,1) = Time(k,1)+del_T(k+1,1);

for l = del_S+1:del_S:j
	del_T(l+1,1) = 2*del_S/(v(l+1,1)+v(l,1));
    Time(l+1,1)= Time(l,1)+del_T(l+1,1);
end

for k = j+1:del_S:S_max
    
    % Calculating time of actual speed (without filter if set separated)
    del_T(k+1,1) = 2*del_S/(vel(k+1,1)+vel(k,1));
    Time(k+1,1)= Time(k,1)+del_T(k+1,1);
    
    % Calculating time of actual speed (with filter if set separated)
    %del_T_kf(i+1,1) = 2*del_S/(v_kf(i+1,1)+v_kf(i,1));
    %Time_kf(i+1,1)= Time_kf(i,1)+del_T_kf(i+1,1);
    
	%jerk(k+1,1) = (accel(k+1,1)-accel(k,1))/(Time(k+1,1)-Time(k,1));
end

for k = 1:del_S:S_max
    % Calculating time of speed reference
    del_T_ref(k+1,1) = 2*del_S/(v_ref_ori(k+1,1)+v_ref_ori(k,1));
    Time_ref(k+1,1)= Time_ref(k,1)+del_T_ref(k+1,1);
end

% figure
% plot(Time_kf./60,s);
% grid on
% title('Running Time (with filter)');
% xlabel('Time (min)');
% ylabel('Distance (m)');
% %axis tight
% maxTime_kf = max(Time_kf)