% Timej = zeros(size(s));
% del_Tj = zeros(size(s));
% dwell = 5;

% Adding 5-second dwelling delay
i = del_S;
del_Tj(i+1,1) = 2*del_S/(v(i+1,1)+v(i,1)) + dwell;
Timej(i+1,1) = Timej(i,1)+del_Tj(i+1,1);

for i = del_S+1:del_S:S_max
    
    % Calculating time of actual speed (without filter if set separated)
    del_Tj(i+1,1) = 2*del_S/(v(i+1,1)+v(i,1));
    Timej(i+1,1)= Timej(i,1)+del_Tj(i+1,1);
    
    % Calculating time of actual speed (with filter if set separated)
    %del_T_kf(i+1,1) = 2*del_S/(v_kf(i+1,1)+v_kf(i,1));
    %Time_kf(i+1,1)= Time_kf(i,1)+del_T_kf(i+1,1);
end

for i = del_S:del_S:S_max
    % Calculating time of speed reference
    del_T_ref(i+1,1) = 2*del_S/(v_ref2(i+1,1)+v_ref2(i,1));
    Time_ref(i+1,1)= Time_ref(i,1)+del_T_ref(i+1,1);

    jerkj(i+1,1) = (acceleration(i+1,1)-acceleration(i,1))/(Timej(i+1,1)-Timej(i,1));
end

maxTimej = max(Timej);
maxTime_ref = max(Time_ref);
% figure
% plot(Time_kf./60,s);
% grid on
% title('Running Time (with filter)');
% xlabel('Time (min)');
% ylabel('Distance (m)');
% %axis tight
% maxTime_kf = max(Time_kf)