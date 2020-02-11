%if vel(i,1) < v_lim(i+1,1)     %compares current vel to next limit
    %vel(i+1,1)=abs((vel(i,1).^2+2*accelj(i+1,1)*(del_S)).^0.5);
    %if vel(i+1,1) > v_lim(i+1,1)
    %    vel(i+1,1) = v_lim(i+1,1);
%     if vel(i+1,1) == v_lim(i+1,1) 
%     end
%else
%     if accelj(i+1,1) < 0
%         vel(i+1,1) = abs((vel(i,1).^2+2*(accelj(i+1,1))*(del_S)).^0.5);
%         if vel(i+1,1) > v_lim(i+1,1)
%             vel(i+1,1) = v_lim(i+1,1);
%         end
%     else
%         accelj(i+1,1) = 0;          % zero acceleration
%         if v_lim(i+1,1) == 0    % if vel_limit = 0
%             vel(i+1,1) = 0;      % then vel = 0
%         else                    % otherwise velF is the same as previous
%             vel(i+1,1) = v_lim(i,1);
%         end
%     end

if vel(i,1) >= v_lim(i+1,1)
    if vel(i,1) <= 15
        Bvj(i,1) = max_brake;
    end
    if vel(i,1) > 15
        Bvj(i,1) = max_brake - (1000*(vel(i,1)-15));
    end
    Bvj(i,1) = Bvj(i,1)*1;
    accj(i,1) = Bvj(i,1)./mass;
    if accj(i,1) < max_decel
        accj(i,1) = max_decel;
    end
    accelj(i+1,1) = -accj(i,1) - accRvj(i,1) - grad_resj(i,1);
    if i == S_max
        accelj(end,1) = 0;
    end
    vel(i+1,1)=abs((vel(i,1).^2+2*accelj(i+1,1)*(del_S)).^0.5);
    flag = flag + 1;
end

if flag == 1
    accelj(i+1,1) = accj(i+1,1) - accRvj(i,1) - grad_resj(i,1);
    if i == S_max
        accelj(end,1) = 0;
    end
    vel(i+1,1)=abs((vel(i,1).^2+2*accelj(i+1,1)*(del_S)).^0.5);
else
    if vel(i+1,1) > v_ref(i+1,1)
        if vel(i,1) <= 15
            Bvj(i,1) = max_brake;
        end
        if vel(i,1) > 15
            Bvj(i,1) = max_brake - (1000*(vel(i,1)-15));
        end
        Bvj(i,1) = Bvj(i,1)*1;
        accj(i,1) = Bvj(i,1)./mass;
        if accj(i,1) < max_decel
            accj(i,1) = max_decel;
        end
        accelj(i+1,1) = -accj(i,1) - accRvj(i,1) - grad_resj(i,1);
        if i == S_max
            accelj(end,1) = 0;
        end
        vel(i+1,1)=abs((vel(i,1).^2+2*accelj(i+1,1)*(del_S)).^0.5);
    else
        vel(i+1,1) = v_ref(i+1,1);
    end
end

if i == S_max
    vel(end,1) = 0;
end

if vel(i+1,1) <= v_B(i+1,1)
    acceler(i+1,1) = accelj(i+1,1);
    v_ref2(i+1,1) = vel(i+1,1);
else
    acceler(i+1,1) = -accel_B(i+1,1);
    v_ref2(i+1,1) = v_B(i+1,1);
end