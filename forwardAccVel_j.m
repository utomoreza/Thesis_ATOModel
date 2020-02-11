accelj(j+1,1) = accj(j+1,1) - accRvj(j,1) - grad_resj(j,1);
if j == S_max
    accelj(end,1) = 0;
end

if vel(j,1) < v_lim(j+1,1)     %compares current vel to next limit
    vel(j+1,1)=abs((vel(j,1).^2+2*accelj(j+1,1)*(del_S)).^0.5);
    if vel(j+1,1) > v_lim(j+1,1)
        vel(j+1,1) = v_lim(j+1,1);
    end
    
else
    if accelj(j+1,1) < 0
        vel(j+1,1) = abs((vel(j,1).^2+2*(accelj(j+1,1))*(del_S)).^0.5);
        if vel(j+1,1) > v_lim(j+1,1)
            vel(j+1,1) = v_lim(j+1,1);
        end
    else
        accelj(j+1,1) = 0;          % zero acceleration
        if v_lim(j+1,1) == 0    % if vel_limit = 0
            vel(j+1,1) = 0;      % then vel = 0
        else                    % otherwise velF is the same as previous
            vel(j+1,1) = v_lim(j,1);
        end
    end
end

if j == S_max
    vel(end,1) = 0;
end