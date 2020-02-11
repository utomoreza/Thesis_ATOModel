clear all
%clc

load s
load v_ref
load v_ref_ori
load v_lim
load gradient
load grad
load u_ref
load ref1
load ref2
load ref3
load add_davis
load angle_Lcurve
load u_noise

% Declare fundamental parameters
L_train = 140;
gravity = 9.81;
mass = 286000;
max_trac = 310000;
max_accel = max_trac/mass;
max_speed = 22.2;
max_brake = 260000;
max_decel = -(max_brake/mass);
Davis = [0.005 0.23 2965];
angle = angle_Lcurve(:,1);
L_curve = angle_Lcurve(:,2);

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

% if you want to test the system using step input
% with step size of increment 1 until reaching value 20,
% uncomment lines below then comment "load v_ref" and "load v_lim"
% v_ref = zeros(5001,1);
% steps = length(v_ref)/(max_speed - 2.2);
% SbyS = (round(1:steps:length(v_ref)))';
% v_ref(1:(SbyS(2,1)),1) = 1;
% for i = 1:18
%     v_ref((SbyS(i+1,1):SbyS(i+2,1)),1) = v_ref(SbyS(i,1),1) + 1;
% end
% v_ref(SbyS(20,1):end,1) = v_ref(SbyS(19,1),1) + 1;
% v_lim = zeros(size(v_ref));
% v_lim(:,1) = max_speed;
% % *************************************************************************
% load v_steps
% v_ref = v_steps;
% v_lim(:,1) = max_speed;
% *************************************************************************

% *************************************************************************
% if you want to use Vref up-down, uncomment lines below
% load ref_updown
% load ref4
% v_ref = ref4;
% v_lim(:,1) = max_speed;
% *************************************************************************

% if you want to test the system using step input of step size of 20,
% uncomment lines below:
% v_ref = zeros(5001,1);
% v_ref(:,1) = max_speed - 2.2;
% v_lim = zeros(size(v_ref));
% v_lim(:,1) = max_speed;
% load u_ref_step
% u_ref = u_ref_step;
% *************************************************************************

% if you want to test the system using step input of step size of 1,
% uncomment lines below:
% v_ref = zeros(5001,1);
% v_ref(:,1) = 1;
% v_lim = zeros(size(v_ref));
% v_lim(:,1) = max_speed;
% *************************************************************************

S_max = length(v_ref)-1;
del_S = S_max/(length(v_ref)-1);
s = 0:del_S:S_max;
s = s';
t = 0:0.1:S_max/10;
t = t';
step = zeros(size(s));
for j = 1:100:S_max
    step(j,1) = j;
end
step(1,1) = 0;

ten = zeros(size(s));
for j = 1:50:S_max+1
    ten(j,1) = j;
end

% u_noise = zeros(size(s));


%altitude = zeros(size(s));
grad_res = zeros(size(s));
grad_resj = zeros(size(s));
error = zeros(size(s));
errorj = zeros(size(s));
v = zeros(size(s));
Fv = zeros(size(s));
Fvj = zeros(size(s));
Bv = zeros(size(s));
Bvj = zeros(size(s));
Rv = zeros(size(s));
Rt = zeros(size(s));
Rc = zeros(size(s));
R_tot = zeros(size(s));
Rvj = zeros(size(s));
Time = zeros(size(s));
Timej = zeros(size(s));
del_T = zeros(size(s));
del_Tj = zeros(size(s));
acc = zeros(size(s));
accj = zeros(size(s));
jerk = zeros(size(s));
jerkj = zeros(size(s));
noise = zeros(size(s));
noise_pure = zeros(size(s));
del_e = zeros(size(s));
del_ej = zeros(size(s));
sig_e = zeros(size(s));
sig_ej = zeros(size(s));
u = zeros(size(s));
uj = zeros(size(s));
v_kf = zeros(size(s));
%del_v = zeros(size(s));
%accBv = zeros(size(s));
%accFv = zeros(size(s));
accRv = zeros(size(s));
accRtot = zeros(size(s));
accRvj = zeros(size(s));
accel = zeros(size(s));
accelj = zeros(size(s));
del_T_ref = zeros(size(s));
Time_ref = zeros(size(s));
v_noNoise = zeros(size(s));
v_noise = zeros(size(s));
u_noNoise = zeros(size(s));
u_noise_pure = zeros(size(s));

v_B = zeros(size(s));
Rv_B = zeros(size(s));
accRv_B = zeros(size(s));
accB = zeros(size(s));
accel_B = zeros(size(s));
acceleration = zeros(size(s));
vel = zeros(size(s));

velo = zeros(size(s));
acceler = zeros(size(s));
v_ref2 = v_ref;

Time_kf = zeros(size(s));
del_T_kf = zeros(size(s));
dwell = 0;

% Declare variables for Kalman filter
Xk_init = [0; 0];
Xinitial = 0;
Xk = [];
Xk_buffer = zeros(2,S_max+1);
Xk_buffer(:,1) = Xk_init;
Z_buffer = zeros(1,S_max+1);
P = eye(2);
I = eye(2);
H = [1 0];
Q = [1e-5^2  1e-5^2;
    1e-5^2   1e-5^2];
W = [1e-5^2;
    1e-5^2];
dt = zeros(size(s));

%gradResistance;

% backwardAccVel;

% Emergency situations
% Event of fire
flag_fire = 0;
fire = zeros(size(s));
% fire_prio = round(rand*1e4);
% if fire_prio > S_max
%     fire_prio = S_max;
% end
% fire(fire_prio,1) = 1;

for j = 1:del_S:S_max
    
    if fire(j,1) == 1
        backwardAccVel;
        max_decel = max_decel - 0.5;
        for i = j:del_S:S_max
            traction;
            Fv(i,1) = Fv(i,1)*1;
            acc(i+1,1) = Fv(i,1)./mass;
            if acc(i+1,1) > max_accel
                acc(i+1,1) = max_accel;
            end
            resistances;
            forwardAccVel;
        end
        for i = j:del_S:S_max
            if v(i,1) <= v_B(i,1)
                acceleration(i,1) = accel(i,1);
                v(i,1) = v(i,1);
            else
                acceleration(i,1) = -accel_B(i,1);
                v(i,1) = v_B(i,1);
            end
        end
        v_ref2(j:end,1) = v(j:end,1);
        flag_fire = 1; 
    else
        if flag_fire == 0
            if j > 999 && j <= 1200
                v_ref(j+1,1) = v_ref(1000,1);
            end
            
            if j > 2000 && j <= 2500
                v_ref(j+1,1) = 18;
            end
            
            if j >= 3500
                if v_ref_ori(j,1) >= 17
                    v_ref(j+1,1) = 17;
                end
            end
            
            if j == 1
                vel = v_ref;
                j = 100;
                %timeJerk;
                k = del_S;
                del_T(k+1,1) = 2*del_S/(vel(k+1,1)+vel(k,1)) + dwell;
                Time(k+1,1) = Time(k,1)+del_T(k+1,1);
                for i = del_S:del_S:S_max
                    del_T(i+1,1) = 2*del_S/(vel(i+1,1)+vel(i,1));
                    Time(i+1,1)= Time(i,1)+del_T(i+1,1);
                    del_T_ref(i+1,1) = 2*del_S/(v_ref_ori(i+1,1)+v_ref_ori(i,1));
                    Time_ref(i+1,1)= Time_ref(i,1)+del_T_ref(i+1,1);
                end
                
                maxTime = Time(end,1);
                maxTime_ref = Time_ref(end,1);
                
                if maxTime <= (maxTime_ref + (maxTime_ref/100*1)) && ...
                        maxTime >= (maxTime_ref - (maxTime_ref/100*1))
                    v_ref2 = vel;
                    %main;
                else
                    j = 1;
                    flag = 1;
                    for i = j:del_S:j+100
                        GenerateFullPower;
                    end
                    v_ref2(j+100+1:end,1) = v_ref(j+100+1:end,1);
                end
                j = 1;
            end
            
            if j == step(j,1)
                %j = 101;
                for i = j:del_S:S_max
                    if vel(i,1) > v_ref(i,1)
                        if vel(i,1) <= 15
                            Bvj(i,1) = max_brake;
                        end
                        if vel(i,1) > 15
                            Bvj(i,1) = max_brake - (1000*(vel(i,1)-15));
                        end
                        Bvj(i,1) = Bvj(i,1)*1;
                        accj(i,1) = Bvj(i,1)./mass;
                        if accj(i,1) < max_decel
                            accj(i,1) = max_decel;
                        end
                        accelj(i+1,1) = -accj(i,1) - accRvj(i,1) - grad_resj(i,1);
                        if i == S_max
                            accelj(end,1) = 0;
                        end
                        vel(i+1,1)=abs((vel(i,1).^2+2*accelj(i+1,1)*(del_S)).^0.5);
                        v_ref2(i+1,1) = vel(i+1,1);
                    else
                        v_ref2(i+1,1) = v_ref(i+1,1);
                    end
                end
                timeJerk;
                maxTime = Time(end,1);
                maxTime_ref = Time_ref(end,1);
                
                if maxTime <= (maxTime_ref + (maxTime_ref/100*1)) && ...
                        maxTime >= (maxTime_ref - (maxTime_ref/100*1))
                    v_ref2(j:end,1) = v_ref2(j:end,1);
                    %main;
                else
                    flag = 1;
                    if j == 4901
                        for i = j:del_S:j+100-1
                            GenerateFullPower;
                        end
                    else
                        for i = j:del_S:j+100
                            GenerateFullPower;
                        end
                    end
                    v_ref2(j+100+1:end,1) = v_ref(j+100+1:end,1);
                end
            end
            
            if j > 999 && j <= 1200
                v_ref2(j+1,1) = v_ref(j+1,1);
            end
            
            if j > 2000 && j <= 2500
                v_ref2(j+1,1) = v_ref(j+1,1);
            end
            
            if j >= 3500
                if v_ref_ori(j,1) >= 17
                    v_ref2(j+1,1) = v_ref(j+1,1);
                end
            end
        else
            flag_fire = 1;
        end
    end
    
    i = j;
    
    error(i,1) = v_ref2(i+1,1) - v(i,1);
    pid_control;
    
%     % applying control signal noise
% %     randomNoise_u;
%     u_noNoise(i,1) = u(i,1);
%     u(i,1) = u(i,1) + u_noise(i+1,1);
%     Q = [u_noise(i+1,1) u_noise(i+1,1);
%         u_noise(i+1,1)  u_noise(i+1,1)];
%     W = [u_noise(i+1,1);
%         u_noise(i+1,1)];
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     % applying process noise
%     u(i,1) = u(i,1) + W(1,1); %1e-5^2;
%     %%%%%%%%%%%%%%%%%%%%%%%%
    
    normalRunning;
        
%     % applying measurement errors
%     randomNoise;
% %     defining known inputs (u) for Kalman filter
%     v_noNoise(i+1,1) = v(i+1,1) / noise(i+1,1);
%     v_noise(i+1,1) = v(i+1,1);
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if i == 1
        dt(i+1,1) = 2*del_S/(v(i+1,1)+v(i,1)) + dwell;
    else
        dt(i+1,1) = 2*del_S/(v(i+1,1)+v(i,1));
    end
    j = i;
    
%      % using Kalman Filter
%     k = i;
%     KalFil;
%     v(k+1,1) = Xk(1,1);
%     if v(k+1,1) > v_lim(k+1,1)
%         v(k+1,1) = v_lim(k+1,1);
%     end
%     j = k;

% 	v(i,1) = kalmanFilter(v_noNoise(i,1), v(i,1), noise_pure(i,1));
%     v(1:7,1) = v_ref2(1:7,1);
end

%         %v(j,1) = vel(j-1,1);
%         for i = j:del_S:S_max
%             error(i,1) = v_ref(i+1,1) - v(i,1);
%             pid_control;
%             if u(i,1) > 0
%                 traction;
%                 Fv(i,1) = Fv(i,1)*u(i,1);
%                 acc(i,1) = Fv(i,1)./mass;
%                 if acc(i,1) > max_accel
%                     acc(i,1) = max_accel;
%                 end
%             else
%                 if u(i,1) < 0
%                     braking;
%                     Bv(i,1) = Bv(i,1)*u(i,1);
%                     acc(i,1) = Bv(i,1)./mass;
%                     if acc(i,1) < max_decel
%                         acc(i,1) = max_decel;
%                     end
%                 else
%                     acc(i,1) = 0;
%                 end
%             end
%             resistances;
%             forwardAccVel;
%         end % end of generating running time
%         
%         timeJerk;
%         maxTime = Time(end,1);
%         maxTime_ref = Time_ref(end,1);
%         
%         if maxTime <= (maxTime_ref + (maxTime_ref/100*3)) && ...
%                 maxTime >= (maxTime_ref - (maxTime_ref/100*3))
%             acceleration(j:S_max,1) = accel(j:S_max,1);
%             vel(j:S_max,1) = v(j:S_max,1);
%         else
%             if maxTime > (maxTime_ref + (maxTime_ref/100*3))
%                 for i = j:del_S:S_max
%                     GenerateFullPower;
%                 end
%                 
%                 for i = j:del_S:S_max
%                     errorj(i,1) = v_ref2(i+1,1) - vel(i,1);
%                     normal_FullPower;
%                     if j > 999 && j <= 1200
%                         %v_ref(j+1,1) = v_ref(1000,1);
%                         if vel(j+1,1) > v_ref(j,1)
%                             vel(j+1,1) = v_ref(j,1);
%                         end
%                     end
%         
%                     if j > 2000 && j <= 2500
%                         %v_ref(j+1,1) = 18;
%                         if vel(j+1,1) > v_ref(j,1)
%                             vel(j+1,1) = v_ref(j,1);
%                         end
%                     end
%     
%                     if j >= 3500
%                         if v_ref_ori(j,1) >= 17
%                             %v_ref(j+1,1) = 17;
%                             if vel(j+1,1) > v(j,1)
%                                 vel(j+1,1) = v(j,1);
%                             end
%                         end
%                     end
%                 end
%                 
%                 
%             else
%                 acceleration(j:S_max,1) = accel(j:S_max,1);
%                 vel(j:S_max,1) = v(j:S_max,1);
%             end
%         end % end of comparing running time
%     else
%         if maxTime <= (maxTime_ref + (maxTime_ref/100*3)) && ...
%                 maxTime >= (maxTime_ref - (maxTime_ref/100*3))
%             acceleration(j:S_max,1) = accel(j:S_max,1);
%             vel(j:S_max,1) = v(j:S_max,1);
%         else
%             errorj(j,1) = v_ref(j+1,1) - vel(j,1);
%             normal;
%         end
%     end % end of step 100 m
% 
% end % end of entire for

% 
% end
% 
% end

%v(1:6,1) = v_noise(1:6,1);

% *************************************************************
% Looping for calculating backward velocity
%backwardAccVel;

% *************************************************************
% Looping for calculating time and jerk
timeJerk_j;

figure
plot(s, v,'b-');
hold on
plot(s, v_ref2,'g-.');
plot(s, v_lim,'k--');
% plot(s,v_noise,'r:');
% plot(s, Xk_buffer(1,:),'r:');
grid on
xlabel('Distance (m)')
ylabel('Speed (m/s)')
title('Speed comparison')
legend('Actual speed','Speed reference','Speed limit')
% axis([0 50 0 23])
% figure,
% plot(Timej,s)
% grid on

% applying lowpass filter
%lowpassFilter;

% plotting important data
%plotting;

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