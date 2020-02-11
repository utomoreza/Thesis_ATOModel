if vel(i,1) <= 10
    Fvj(i,1) = max_trac; 
end
if vel(i,1) > 10 && vel(i,1) <= 22.2
    Fvj(i,1) = max_trac - (1000*(vel(i,1)-10));
end

Fvj(i,1) = Fvj(i,1)*1;
accj(i+1,1) = Fvj(i,1)./mass;
if accj(i+1,1) > max_accel
    accj(i+1,1) = max_accel;
end

% uncomment line below to apply gradient profile down to up
%grad_resj(i,1) = gradient(i,1)*gravity;

% uncomment line below to apply gradient profile up to down
%grad_resj(i,1) = gradient(i,2)*gravity;

% uncomment line below to apply gradient profile zero
grad_resj(i,1) = 0*gravity;

Rvj(i,1) = Davis(1)*vel(i,1).^2 + Davis(2)*vel(i,1) + Davis(3);
accRvj(i,1) = Rvj(i,1)./mass;

FwdBwdAccVel;