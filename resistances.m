% uncomment line below to apply gradient profile down to up
% grad_res(i,1) = gradient(i,1)*gravity/10;

% uncomment line below to apply gradient profile up to down
% grad_res(i,1) = sin(gradient(i,2))*gravity/10;

% uncomment line below to apply gradient profile zero
% grad_res(i,1) = 0*gravity;

% uncomment line below to apply more realistic gradient profile
% type 1
% grad_res(i,1) = grad(i,1)./100*gravity;
% type 2
% grad_res(i,1) = grad(i,2)./100*gravity;

% running resistance
Rv(i,1) = Davis(1)*v(i,1).^2 + Davis(2)*v(i,1) + Davis(3);

% running resistance -- using noise
% Rv(i,1) = Davis(1)*add_davis(i,1)*v(i,1).^2 + Davis(2)*add_davis(i,2)*v(i,1) + Davis(3)*add_davis(i,3);

% tunnel resistance
% when 300 < s <= 1000 AND 4400 < s <= 4900
% if i > 300 && i <= 1300
%     %Rt(i,1) = 0.00013*L_train; %v(i,1).^2*L_train/10^7;
%     Rt(i,1) = 7.7*v(i,1);
% end
% if i > 2500 && i <= 3500
%     Rt(i,1) = 7.7*v(i,1);
% end
% if i > 4400 && i <= 4900
%     %Rt(i,1) = 0.00013*L_train; %v(i,1).^2*L_train/10^7;
%     Rt(i,1) = 7.7*v(i,1);
% end

% curvature resistance
% when 100 < s <= 250 AND 3000 < s <= 4000 AND 4400 < s <= 4900
% if L_curve(i,1) >= L_train
%    Rc(i,1) = 10.5*angle(i,1)./L_curve(i,1); % if L_curve >= Ltrain
% end
% if L_curve(i,1) <= L_train
%    Rc(i,1) = 10.5*angle(i,1)./L_train; % if L_curve <= Ltrain
% end

% total resistances
R_tot(i,1) = Rv(i,1) + Rc(i,1) + Rt(i,1);
% R_tot(i,1) = Rv(i,1)./mass;
accRtot(i,1) = R_tot(i,1)./mass + grad_res(i,1); % + Rt(i,1);
% accRtot(i,1) = R_tot(i,1) + grad_res(i,1);
% accRtot(1,1) = 0;
% accRtot(end,1) = 0;