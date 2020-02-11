% i = del_S;
% del_Tj(i+1,1) = 2*del_S/(v(i+1,1)+v(i,1)) + dwell;
% Timej(i+1,1) = Timej(i,1)+del_Tj(i+1,1);

for i = del_S:del_S:S_max
    % Calculating time of actual speed (without filter if set separated)
    del_Tj(i+1,1) = 2*del_S/(v(i+1,1)+v(i,1));
    Timej(i+1,1)= Timej(i,1)+del_Tj(i+1,1);
end

v_try = zeros(size(s));
for t = del_S:S_max
    v_try(t+1,1) = v_try(t,1) + acceleration(t,1).*del_Tj(t+1,1);
%     v_try(t+1,1) = abs((v_try(t,1).^2 + 2*acceleration(t,1).*(del_S)).^0.5);
end
v_try(t+1,1) = 0;

figure
plot(s,v_try,'b-')
hold on
plot(s,v,'g-.')
plot(s,v_lim,'k--')
grid on
title('Comparison of two type of model')
xlabel('Distance (m)')
ylabel('Speed (m/s)')
legend('Actual speed from KF model','Actual speed from control model',...
    'Speed limit')

v_min_vtry = v - v_try;
e_1percent_pos = v./100*1;
over2_pos = v./100*2;
e_1percent_neg = -e_1percent_pos;
over2_neg = -over2_pos;
figure,
plot(s,v_min_vtry,'k-')
hold on
plot(s,e_1percent_pos,'b--');
plot(s,e_1percent_neg,'b--');
plot(s,over2_pos,'r--')
plot(s,over2_neg,'r--')
grid on
title('Comparison of error deviation between 2 types of model')
xlabel('Distance (m)')
ylabel('Speed-error (m/s)')
legend('Error deviation','+1% error limit','-1% error limit',...
    '+2% error limit','-2% error limit')
axis tight

% YOU SHOULD TRY TO MODEL EQUATION V(T+1) = V(T) + AT
% FROM BRAKING (BACKWARD) APPROACH