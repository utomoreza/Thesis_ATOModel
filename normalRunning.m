if u(i,1) > 0
    traction;    
    Fv(i,1) = Fv(i,1)*u(i,1);   
    acc(i+1,1) = Fv(i,1)./mass;   
    if acc(i+1,1) > max_accel    
        acc(i+1,1) = max_accel;        
    end    
else    
    if u(i,1) < 0    
        braking;        
        Bv(i,1) = Bv(i,1)*u(i,1);        
        acc(i+1,1) = Bv(i,1)./mass;        
        if acc(i+1,1) < max_decel        
            acc(i+1,1) = max_decel;           
        end        
    else
        acc(i+1,1) = 0;        
    end    
end
resistances;
forwardAccVel;
acceleration(i,1) = accel(i,1);
% acceleration(1,1) = 0;
% acceleration(end,1) = 0;