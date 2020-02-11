% PID controller
% u = P + I*(1/s) + D*(N/1+(N*(1/s)));

% best parameters to date
% P = 5;
% I = 1e-4;
% D = 0.1;
% N = 1;

% P = 40; %5;
% I = 0; %1e-4;
% D = 1; %0.5;
% N = 1;

 del_ej(j,1) = (errorj(j+1,1) - errorj(j,1));%/(t(j+1,1)-t(j,1));
 del_ej(1,1) = 0;
 sig_ej(1,1) = 0;
 sig_ej(j+1,1) = (sig_ej(j,1) + errorj(j,1));%/(t(j+1,1)-t(j,1));
 
% u(j,1) = errorj(j,1).*P + sig_ej(j,1).*I + D*(N/1+(N*(sig_ej(j,1))));

%%
% *******************************************************
% Simple PID controller
kp = 42;
ki = 1e-5;
kd = 25;

uj(j,1) = errorj(j,1).*kp + sig_ej(j,1).*ki + del_ej(j,1).*kd;
% *******************************************************

%%
% *******************************************************
% Limit PID output signal
if uj(j,1) > 1
	uj(j,1) = 1;
end

if uj(j,1) < -1
	uj(j,1) = -1;
end