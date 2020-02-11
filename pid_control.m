% PID controller
% u = P + I*(1/s) + D*(N/1+(N*(1/s)));

%%
% Prop(i+1,1) = error(i+1,1);% error of proportional term
% Der(i+1,1)  = (error(i+1) - error(i)); % derivative of the error
Int(i+1,1)  = Int(i,1) + (error(i+1) + error(i))./2; % integration of the error
% I(i+1,1)    = sum(Int); % the sum of the integration of the error

% PID(i+1)  = kp*Prop(i) + ki*I(i+1)+ kd*Der(i); % the three PID terms
% u(i+1)  = kp*Prop(i) + kp/ti*Int(i+1)+ kp*td*Der(i); % the three PID terms
%%

% best parameters to date
% P = 5;
% I = 1e-4;
% D = 0.1;
% N = 1;

% P = 40; %5;
% I = 0; %1e-4;
% D = 1; %0.5;
% N = 1;

% if i == 1
%     del_e(i,1) = 0; %error(i,1) - 0;
% else
%     del_e(i,1) = error(i,1) - error(i-1,1);
% end

%  del_e(1,1) = 0;
%  sig_e(1,1) = 0;
%  if i == 1
%      sig_error(i+1,1) = error(i+1,1) + error(i,1);
%  else
%      sig_error(i+1,1) = (sig_error(i,1) + error(i+1,1));%/(t(i+1,1)-t(i,1));
%  end
  
% sig_e(i,1) = trapz(error);
%  sig_e(i,1) = trapz()
 
% u(i,1) = error(i,1).*P + sig_e(i,1).*I + D*(N/1+(N*(sig_e(i,1))));

% u(i,1) = error(i,1).*kp + del_e(i,1).*kd + sig_e(i,1).*ki;

% u = kp * (e + 1/ti * sig_e + td * del_e)
Prop(i,1) = kp.*error(i,1);
% Integ(i,1) = kp/ti.*sig_e(i,1);%.*dt(i+1,1);
Der(i,1) = kp*td.*del_e(i,1);%./dt(i+1,1);%
u(i,1) = Prop(i,1) + Der(i,1) + Integ(i,1);
u_nolimit(i,1) = u(i,1);
% *******************************************************
% Limit PID output signal
if u(i,1) > 1
	u(i,1) = 1;
end

if u(i,1) < -1
	u(i,1) = -1;
end
% BACK-CALCULATION BASED ANTI-WINDUP
% kw = 0.6825;
% u_min_v = u(i,1)-u_nolimit(i,1);
% if u_min_v < 0 || u_min_v > 0
%     Prop(i,1) = (kp + u_min_v*0.6825).*error(i,1); %Prop(i,1) + u_min_v/(1);
%     Integ(i,1) = (kp/ti + u_min_v*0.5).*sig_e(i,1); %Integ(i,1) + u_min_v/(ti+20);
%     u(i,1) = Prop(i,1) + Der(i,1) + Integ(i,1);
%     u_nolimit(i,1) = u(i,1);
% end
% *******************************************************
% Limit PID output signal
% if u(i,1) > 1
% 	u(i,1) = 1;
% end
% 
% if u(i,1) < -1
% 	u(i,1) = -1;
% end