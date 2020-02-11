% v_B = zeros(size(s));
% Rv_B = zeros(size(s));
% accRv_B = zeros(size(s));
% accB = zeros(size(s));
% accel_B = zeros(size(s));
% acceleration = zeros(size(s));
% vel = zeros(size(s));

i = S_max;
Rv_B(i+1,1) = Davis(1)*v_B(i+1,1).^2 + Davis(2)*v_B(i+1,1) + Davis(3);
accRv_B(i+1,1) = Rv_B(i+1,1)./mass;
accel_B(i+1,1) = brake(v_B(i+1,1), max_brake, -max_decel, mass, coasting, ...
    coasting_vel) + accRv_B(i+1,1) + grad_res(i+1,1);
v_B(i,1) = abs((v_B(i+1,1)^2+2*accel_B(i+1,1)*(del_S))^0.5);

for i = S_max-1:-del_S:1
	if v_B(i+1,1) < v_lim(i+1,1)
        Rv_B(i+1,1) = Davis(1)*v_B(i+1,1).^2 + Davis(2)*v_B(i+1,1) + Davis(3);
        accRv_B(i+1,1) = Rv_B(i+1,1)./mass;
        %if error(route,1) > 0
        %braking;
        %Bv(route,1) = Bv(i,1);%*error(i,1);
        %accB(i+1,1) = Bv(i,1)./mass;
%         if accB(i+1,1) > max_brake/mass
        %    accB(i+1,1) = max_brake/mass;
        %end
        accel_B(i+1,1) = brake(v_B(i+1,1), max_brake, -max_decel, mass, coasting, ...
            coasting_vel) + accRv_B(i+1,1) + grad_res(i+1,1);
        v_B(i,1) = abs((v_B(i+1,1)^2+2*accel_B(i+1,1)*(del_S))^0.5);
        %end
    else
        v_B(i,1) = v_lim(i+1,1);
	end
end   
    
% for i = 1:del_S:S_max
% 	if v(i,1) <= v_B(i,1)
%         acceleration(i,1) = accel(i,1);
%         vel(i,1) = v(i,1);
%     else
%         acceleration(i,1) = -accel_B(i,1);
%         vel(i,1) = v_B(i,1);
% 	end
% end
%     
% figure,
% plot(s,accel_B);
% title('Backward acceleration')
% 
% figure,
% plot(s,v_B.*3.6)
% title('Backward velocity');
% 
% figure,
% plot(s,acceleration);
% title('Acceleration');
% 
% load v_ref
% figure,
% plot(s,vel.*3.6);
% hold on;
% plot(s,v_ref.*3.6);
% title('Speed');
% legend('actual speed','Speed reference');