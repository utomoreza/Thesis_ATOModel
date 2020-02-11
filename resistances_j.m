% uncomment line below to apply gradient profile down to up
%grad_resj(j,1) = gradient(j,1)*gravity;

% uncomment line below to apply gradient profile up to down
%grad_resj(j,1) = gradient(j,2)*gravity;

% uncomment line below to apply gradient profile zero
grad_resj(j,1) = 0*gravity;

Rvj(j,1) = Davis(1)*vel(j,1).^2 + Davis(2)*vel(j,1) + Davis(3);
accRvj(j,1) = Rvj(j,1)./mass;