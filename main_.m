%clear all
%clc

load s
load v_ref
load v_ref_ori
load v_lim
load gradient

gravity = 9.81;
mass = 286000;
max_trac = 310000;
max_accel = max_trac/mass;
max_speed = 22.2;
max_brake = 260000;
max_decel = -(max_brake/mass);
Davis = [0.005 0.23 2965];

% if you want to test the system using step input,
% uncomment 4 lines below then comment "load v_ref", "load v_ref_ori" and "load v_lim"
% v_ref = zeros(5001,1);
% v_ref(:,1) = 10;
% v_lim = zeros(size(s));
% v_lim(:,1) = v_ref(:,1) + 1;

% if you want to use the system without any signal reference,
% uncomment lines below
%v_ref = zeros(5001,1);
%v_ref(1:226,1) = 10;
%v_ref(227:1001,1) = 15;
%v_ref(1002:4350,1) = 20; %v_ref(1002:4176,1) = 20;
%v_ref(4351:5000,1) = 10;
%v_ref(end,1) = 0;
%v_lim = zeros(size(s));
%v_lim(:,1) = v_ref(:,1);

S_max = length(v_ref)-1;
del_S = S_max/(length(v_ref)-1);
s = 0:del_S:S_max;
s = s';
t = 0:0.1:S_max/10;
t = t';

%altitude = zeros(size(s));
grad_res = zeros(size(s));
error = zeros(size(s));
v = zeros(size(s));
Fv = zeros(size(s));
Bv = zeros(size(s));
Rv = zeros(size(s));
Time = zeros(size(s));
del_T = zeros(size(s));
acc = zeros(size(s));
jerk = zeros(size(s));
noise = zeros(size(s));
del_e = zeros(size(s));
sig_e = zeros(size(s));
u = zeros(size(s));
v_kf = zeros(size(s));
%del_v = zeros(size(s));
%accBv = zeros(size(s));
%accFv = zeros(size(s));
accRv = zeros(size(s));
accel = zeros(size(s));
del_T_ref = zeros(size(s));
Time_ref = zeros(size(s));
v_noNoise = zeros(size(s));
v_noise = zeros(size(s));

%gradResistance;

%%
% *************************************************************
% Looping for calculating forward velocity
for i = 1:del_S:S_max
    %%
    % *********************************************************
    % Implementing 3 different running time delays
    % (When step response is applied, comment if statements below)
%     if i > 999 && i <= 1200
%         v_ref(i+1,1) = v_ref(1000,1);
%     end
%         
%     if i > 2000 && i <= 2500
%         v_ref(i+1,1) = 18;
%     end
%     
%     if i >= 3500
%         if v_ref_ori(i,1) >= 17
%             v_ref(i+1,1) = 17;
%         end
%     end
    % *********************************************************
    %%
    
    error(i,1) = v_ref(i+1,1) - v(i,1);
	
    %%
    % *********************************************************
    % when the controller is not applied:
    % 1. comment pid_control;
    pid_control;
    
    % 2. uncomment two if statements below
% 	if error(i,1) > 1
%        error(i,1) = 1;
%     end
% 
%     if error(i,1) < -1
%        error(i,1) = -1;
%     end
    
    % 3. change u with error
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
    % *********************************************************
    %%
    
    resistances;

    forwardAccVel;
    
    %**********************************************************
    % calculating time and jerk
	%del_T(i+1,1) = 2*del_S/(v(i+1,1)+v(i,1));
    %Time(i+1,1)= Time(i,1)+del_T(i+1,1);

    %del_T_ref(i+1,1) = 2*del_S/(v_ref(i+1,1)+v_ref(i,1));
    %Time_ref(i+1,1)= Time_ref(i,1)+del_T_ref(i+1,1);

    %jerk(i+1,1) = (accel(i+1,1)-accel(i,1))/(Time(i+1,1)-Time(i,1));
    %**********************************************************
    
    % using delta velocity
    %del_v(i,1) = (v(i+1,1)-v(i,1))/(Time(i+1,1)-Time(i,1));
    %del_v(1,1) = 0;
    
    % applying measurement errors
%     randomNoise;
%     % defining known inputs (u) for Kalman filter
%     v_noNoise(i,1) = v(i,1) / noise(i,1);
%     v_noise(i,1) = v(i,1);
%     % using Kalman Filter
% 	v(i,1) = kalmanFilter(v_noNoise(i,1), v(i,1), (noise(i,1)-1));
    %v(1:6,1) = v_noise(1:6,1);
    %[kalmf,L,P,M] = kalman(Plant,Q,R);
end

%%
%v(1:6,1) = v_noise(1:6,1);

% *************************************************************
% Looping for calculating backward velocity
%backwardAccVel;

% *************************************************************
% Looping for calculating time and jerk
timeJerk;

% applying lowpass filter
%lowpassFilter;

% plotting important data
plotting;

%figure(5)
%plot(s./1000, Jerk);
%grid on

%clear i
%accel_kf = zeros(size(s));
%for i = 1:del_S:S_max
%    accel_kf(i+1,1) = ((v_kf(i+1,1)).^2 - (v_kf(i,1)).^2)./2*del_S;
%end
%figure, plot(s, accel_kf)

% figure, plot(s, [v v_kf].*3.6)
% grid on;
% hold on, plot(s, v_ref.*3.6);
% legend('without filter', 'with filter', 'ref');