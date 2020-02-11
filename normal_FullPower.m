pid_control_FullPower;
if uj(i,1) > 0
    if vel(i,1) <= 10
        Fvj(i,1) = max_trac;
    end

    if vel(i,1) > 10 && vel(i,1) <= 22.2
        Fvj(i,1) = max_trac - (1000*(vel(i,1)-10));
    end
    Fvj(i,1) = Fvj(i,1)*uj(i,1);   
    accj(i+1,1) = Fvj(i,1)./mass;   
    if accj(i+1,1) > max_accel    
        accj(i+1,1) = max_accel;        
    end    
else    
    if uj(i,1) < 0    
        if vel(i,1) <= 15
            Bvj(i,1) = max_brake;
        end

        if vel(i,1) > 15
            Bvj(i,1) = max_brake - (1000*(vel(i,1)-15));
        end
        Bvj(i,1) = Bvj(i,1)*uj(i,1);        
        accj(i+1,1) = Bvj(i,1)./mass;        
        if accj(i+1,1) < max_decel        
            accj(i+1,1) = max_decel;           
        end        
    else
        accj(i+1,1) = 0;        
    end    
end

% uncomment line below to apply gradient profile down to up
%grad_resj(i,1) = gradient(i,1)*gravity;

% uncomment line below to apply gradient profile up to down
%grad_resj(i,1) = gradient(i,2)*gravity;

% uncomment line below to apply gradient profile zero
grad_resj(i,1) = 0*gravity;

Rvj(i,1) = Davis(1)*vel(i,1).^2 + Davis(2)*vel(i,1) + Davis(3);
accRvj(i,1) = Rvj(i,1)./mass;

accelj(i+1,1) = accj(i+1,1) - accRvj(i,1) - grad_resj(i,1);
if i == S_max
    accelj(end,1) = 0;
end

if vel(i,1) < v_lim(i+1,1)     %compares current vel to next limit
    vel(i+1,1)=abs((vel(i,1).^2+2*accelj(i+1,1)*(del_S)).^0.5);
    if vel(i+1,1) > v_lim(i+1,1)
        vel(i+1,1) = v_lim(i+1,1);
    end
    
else
    if accelj(i+1,1) < 0
        vel(i+1,1) = abs((vel(i,1).^2+2*(accelj(i+1,1))*(del_S)).^0.5);
        if vel(i+1,1) > v_lim(i+1,1)
            vel(i+1,1) = v_lim(i+1,1);
        end
    else
        accelj(i+1,1) = 0;          % zero acceleration
        if v_lim(i+1,1) == 0    % if vel_limit = 0
            vel(i+1,1) = 0;      % then vel = 0
        else                    % otherwise velF is the same as previous
            vel(i+1,1) = v_lim(i,1);
        end
    end
end

if i == S_max
    vel(end,1) = 0;
end

acceleration(i,1) = accelj(i,1);