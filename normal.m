pid_control_j;
if uj(j,1) > 0
    traction_j;    
    Fvj(j,1) = Fvj(j,1)*uj(j,1);   
    accj(j+1,1) = Fvj(j,1)./mass;   
    if accj(j+1,1) > max_accel    
        accj(j+1,1) = max_accel;        
    end    
else    
    if uj(j,1) < 0    
        braking_j;        
        Bvj(j,1) = Bvj(j,1)*uj(j,1);        
        accj(j+1,1) = Bvj(j,1)./mass;        
        if accj(j+1,1) < max_decel        
            accj(j+1,1) = max_decel;           
        end        
    else
        accj(j+1,1) = 0;        
    end    
end
resistances_j;
forwardAccVel_j;
acceleration(j,1) = accelj(j,1);