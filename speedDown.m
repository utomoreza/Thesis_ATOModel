if vel(j,1) <= 15
    Bvj(j,1) = max_brake;
end
if vel(j,1) > 15
    Bvj(j,1) = max_brake - (1000*(vel(j,1)-15));
end

Bvj(j,1) = Bvj(j,1)*1;        
accj(j+1,1) = Bvj(j,1)./mass;        
if accj(j+1,1) < max_decel        
    accj(j+1,1) = max_decel;           
end      

% i = j+100;
% Rv_B(i+1,1) = Davis(1)*v_B(i+1,1).^2 + Davis(2)*v_B(i+1,1) + Davis(3);
% accRv_B(i+1,1) = Rv_B(i+1,1)./mass;
% accel_B(i+1,1) = brake(v_B(i+1,1), max_brake, -max_decel, mass) + accRv_B(i,1) + grad_res(i,1);
% v_B(i,1)=(v_B(i+1,1)^2+2*accel_B(i+1,1)*(del_S))^0.5;
% 
% for i = j+100-1:-del_S:j
% 	if v_B(i+1,1) < v_lim(i+1,1)
%         Rv_B(i+1,1) = Davis(1)*v_B(i+1,1).^2 + Davis(2)*v_B(i+1,1) + Davis(3);
%         accRv_B(i+1,1) = Rv_B(i+1,1)./mass;
%         accel_B(i+1,1) = brake(v_B(i+1,1), max_brake, -max_decel, mass) + accRv_B(i,1) + grad_res(i,1);
%         v_B(i,1)=(v_B(i+1,1)^2+2*accel_B(i+1,1)*(del_S))^0.5;
%     else
%         v_B(i,1) = v_lim(i+1,1);
% 	end
% end