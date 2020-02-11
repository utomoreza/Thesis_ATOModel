function accB = brake(vel, max_brake, max_decel, mass, coasting, ...
    coasting_vel)
if coasting == 1
    if vel >= (coasting_vel/3.6)
        accB = 0;
    else
%         if vel <= 15
            Bv = max_brake;
%         end
        
%         if vel > 15
%             Bv = max_brake - (1000*(vel-15));
%         end
        
        accB = Bv/mass;
        if accB > abs(max_decel)
            accB = abs(max_decel);% max_decel;
        end
    end
else
%     if vel <= 15
        Bv = max_brake;
%     end
    
%     if vel > 15
%         Bv = max_brake - (1000*(vel-15));
%     end
    
    accB = Bv/mass;
    if accB > abs(max_decel)
        accB = abs(max_decel);
    end
end
end