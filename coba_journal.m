clear all
%clc

load s
load v_ref
load v_ref_ori
load v_lim
% load gradient
load grad
% load u_ref
% load ref1
% load ref2
% load ref3
% load add_davis
load angle_Lcurve
% load u_noise
% load accel_ref
% load jerk_ref
% load route_Docklands_MTS_comparison
% load route_Stratford_MoorStreet
% load vel_DLR
% load vel_stratford_moorstreet
% v_ref2 = vel';

fuz_alpha = readfis('Fuzzy-Adaptation');
% figure,plotfis(fuz_sch1)
% figure,plotfis(fuz_sch)
% figure,plotfis(fuz_alpha)
% figure,plotmf(fuz_alpha,'input',1)
% figure,plotmf(fuz_alpha,'input',2)
% figure,plotmf(fuz_alpha,'output',1)

% Declare fundamental parameters
L_train = 140;
gravity = 9.81;
mass = 286000;
max_trac = 310000;
max_accel = max_trac/mass;
max_speed = 22.2;
max_brake = 310000; %260000;
max_decel = -(max_brake/mass);
Davis = [0.005 0.23 2965];
angle = angle_Lcurve(:,1);
L_curve = angle_Lcurve(:,2);


% UNCOMMENT LINES BELOW TO USE S = 5001
% S_max = length(v_ref)-1;
% del_S = S_max/(length(v_ref)-1);
% s = 0:del_S:S_max;
% s = s';
% t = 0:0.1:S_max/10;
% t = t';
% step = zeros(size(s));
% for j = 1:100:S_max
%     step(j,1) = j;
% end
% step(1,1) = 0;
% ten = zeros(size(s));
% for j = 1:50:S_max+1
%     ten(j,1) = j;
% end
% grad_res = zeros(size(s));


% % ADD DLR B2007 STOCK DATA
DLR_vehicle;
% ADD DLR ROUTE DATA
DLR_Route;
S_max = length(s)-1;
del_S = 1;

% grad_res = zeros(size(s));

% *****************************************
% % CHOOSE TRACTION MODEL
% choose_trac = 0; % for model from (Su,2016)
choose_trac = 1; % for DLR model
% *****************************************


% ********************************************************************
% DEFINE TRACK DOMAIN S as the distance variable
% USING DATA FROM STS FILES
% S_min=0;
% S_max=fix(max(vel_profile(:,1))*1000);
% 
% del_S=1;            %This is the distance step keep at 1 metre
% s=S_min:del_S:S_max;
% s = s';
% v_lim = zeros(size(s));


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
% uncomment lines below
% v_ref = zeros(5001,1);
% steps = length(v_ref)/(max_speed - 2.2);
% SbyS = (round(1:steps:length(v_ref)))';
% v_ref(1:(SbyS(2,1)),1) = 1;
% for i = 1:(length(SbyS)-2)
%     v_ref((SbyS(i+1,1):SbyS(i+2,1)),1) = v_ref(SbyS(i,1),1) + 1;
% end
% v_ref(SbyS(end,1):end,1) = v_ref(SbyS(i+1,1),1) + 1;
% v_lim = zeros(size(v_ref));
% v_lim(:,1) = max_speed;
% % *************************************************************************
% load v_steps
% v_steps(1:1298) = 6;
% v_ref = v_steps;
% v_ref_ori = v_steps;
% v_lim(:,1) = max_speed;
% v_ref2 = v_ref;
% *************************************************************************

% *************************************************************************
% if you want to use Vref up-down, uncomment lines below
% load ref_updown
% load ref4
% v_ref = ref4;
% v_ref_ori = ref4;
% v_lim(:,1) = max_speed;
% v_ref2 = v_ref;
% *************************************************************************

% if you want to test the system using step input of step size of 20,
% uncomment lines below:
% v_ref = zeros(5001,1);
% v_ref(:,1) = max_speed - 2.2;
% v_lim = zeros(size(v_ref));
% v_lim(:,1) = max_speed;
% load u_ref_step
% u_ref = u_ref_step;
% v_ref(1:350,1) = 20;
% v_ref(351:end,1) = 5;
% v_ref_ori = v_ref;
% v_ref2 = v_ref;
% *************************************************************************

% if you want to test the system using step input of step size of 1,
% uncomment lines below:
% v_ref = zeros(5001,1);
% v_ref(:,1) = 1;
% v_lim = zeros(size(v_ref));
% v_lim(:,1) = max_speed;
% v_ref_ori = v_ref;
% *************************************************************************

% if you want to test the system using up-ramp input,
% uncomment lines below:
% v_ref = (0:1:length(v_ref))';
% v_lim = zeros(size(v_ref));
% v_lim(:,1) = max_speed;
% v_ref_ori = v_ref;
% *************************************************************************

% if you want to test the system using down-ramp input,
% uncomment lines below:
% for i = length(v_ref):-1:1
%     v_ref(i,1) = length(v_ref) - i;
% end
% v_ref(1:4982,1) = max_speed - 2.2;
% v_lim = zeros(size(v_ref));
% v_lim(:,1) = max_speed;
% v_ref_ori = v_ref;
% *************************************************************************

% u_noise = zeros(size(s));
%altitude = zeros(size(s));
% grad_res = zeros(size(s));
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
u_nolimit = zeros(size(s));
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
% vel = zeros(size(s));

% FUZZY ALPHA
fuzzy_in = zeros(length(s),3);
h = zeros(size(s));
alpha = zeros(size(s));
% ******************************************

velo = zeros(size(s));
acceler = zeros(size(s));
% v_ref2 = v_ref;

v_ref_1stdiff = zeros(size(s));
v_ref_2nddiff = zeros(size(s));

Time_kf = zeros(size(s));
del_T_kf = zeros(size(s));
dwell = 0;
% v_lim(:,1) = max_speed;
TrMinTref = zeros(size(s));

% ********************************************************************
% RUNNING RELAY-BASED CONTROL TO DEFINE
% CRITICAL GAIN Kcr AND PERIOD Pcr
% v_lim(:,1) = max_speed;
v_ref = zeros(size(s));
v_ref(:,1) = 3; %max_speed/2;
relay_amp = 1;
for i = 1:S_max
    error(i,1) = v_ref(i+1,1) - v(i,1);
    if error(i,1) > 0
        u(i,1) = relay_amp;
    end
    if error(i,1) < 0
        u(i,1) = -relay_amp;
    end
    normalRunning;
    del_T(i+1,1) = 2*del_S/(v(i+1,1)+v(i,1));
    Time(i+1,1)= Time(i,1)+del_T(i+1,1);
end
% 4*d/(pi*a)
% FIND AMPLITUDE
pos_d = findpeaks(u);
neg_d = findpeaks(-u);
avg_pos_d = mean(pos_d);
avg_neg_d = mean(neg_d);
d = (avg_pos_d - (-avg_neg_d))/2;

% error_peaks_pos = findpeaks(error);
% error_peaks_neg = findpeaks(-error);
% max_error_peak = max(error_peaks_pos);
% min_error_peak = max(error_peaks_neg);
% amp = (max_error_peak - (-min_error_peak))/2;
pos_peaks = findpeaks(v);
neg_peaks = findpeaks(-v);
max_pos_peaks = max(pos_peaks);
max_neg_peaks = max(neg_peaks);
amp = ((max_pos_peaks) - (-max_neg_peaks))/2;
kcr = 4*d/(pi*amp);
% FIND PERIOD
% Fs = 1/mean(del_T);
% u_dft = abs(fft(error));
% [maxval,idx] = max(abs(u_dft));
% freq = (Fs*(idx-1))/(length(u_dft));
[~,idx] = findpeaks(error);
freq = length(idx)/(max(Time)-Time(idx(1)));
pcr = 1/freq;
% CLEAR VALUE OF IMPORTANT VARIABLES
% v_lim(:,1) = 0;
v(:,1) = 0; u(:,1) = 0; error(:,1) = 0;
Fv(:,1) = 0; Bv(:,1) = 0; acc(:,1) = 0; accRtot(:,1) = 0;
accel(:,1) = 0; acceleration(:,1) = 0; Rv(:,1) = 0;
% ********************************************************************


% *******************************************************
% DEFINING PID PARAMETERS
Prop = zeros(size(s));
Der = zeros(size(s));
Int = zeros(size(s));
% Simple PID controller parameters
% Manual tuning
% kp = 42; %for StepSize1 = 1; %for StepSize20 = 42;
% ki = 1e-5; %for StepSize1 = 1e-2;%StepSize20 = 1e-5;
% kd = 25; %StepSize1 = 0;%StepSize20 = 25;
% kp = 1;%0.475;
% ki = 1;%0.03;
% kd = 1;%0.5;

% kcr = 7.4941; %19; %21; %0.1599; %0.1386; %10; % critical gain (constant oscillation) for StepSize1: 2.1;
% for StepSize20 = ;
% pcr = 0.6693; %0.337; %0.59; % critical period for StepSize20: 1.942; 
% for StepSize20 = ;
% L = 0.1;
% T = 18.8 - L;
% **********************************************
%%%%%%%%%%%%%% finding kcr
% kp = 19;
% ti = inf;
% td = 0;
%%%%%%%%%%%%%% PI controller
% kp = 0.45*kcr;
% ti = pcr/1.2;
% td = 0;
%%%%%%%%%%%%%% PD controller
% kp = 0.8*kcr;
% ti = inf;
% td = pcr/8;
%%%%%%%%%%%%%% Ziegler-Nichols classic
kp = 0.6*kcr;
ti = pcr/2;
td = pcr/8;
%%%%%%%%%%%%%% Ziegler-Nichols no-overshoot
% kp = 0.2*kcr;
% ti = pcr/2;
% td = pcr/3;
%%%%%%%%%%%%%% Ziegler-Nichols some overshoots
% kp = 0.33*kcr;
% ti = inf; %0.5*pcr;
% td = pcr/3;
%%%%%%%%%%%%%% Tyreus-Luyben
% kp = kcr/2.2; 
% ti = 2.2*pcr;
% td = pcr/6.3;
%%%%%%%%%%%%%% Relay feedback approach
% kcr = (4*1)/(pi*0.05);
% tcr = 0.2;
% kp = 0.2*kcr;
% ti = 0.5*tcr;
% td = tcr/3;
%%%%%%%%%%%%%% Relay with hysteresis
relay_amp = 1;
% beta = 1;
% error_max = 2;
% error_min = error_max;
% del_one = beta*error_min;
% del_two = beta*error_max;
% amp = 0.1699; %2.05; %0.1386;
% pc = 0.6693; %8.4090; %0.64;
% kp = 0.33*4*1/(pi*amp);
% ti = 0.7*pc;
% td = 0; %0.12*pc;
% Fs = 1/mean(del_Tj);
% error_dft = fft(error);
% [maxval,idx] = max(abs(error_dft));
% freq = (Fs*(idx-1))/length(error);
% period = 1/freq;
% error_peaks = findpeaks(error);
% avg_error_peak = mean(error_peaks);
% **********************************************
kp_buffer = zeros(size(s));
ti_buffer = zeros(size(s));
td_buffer = zeros(size(s));
Prop = zeros(size(s));
Integ = zeros(size(s));
Der = zeros(size(s));

% Declare variables for Kalman filter
one = ones(size(s));
noise_percent = awgn(one,37,'measured');
% figure,plot(noise_percent)
for n = 1:length(one)
    if noise_percent(n,1) > 1.05
        noise_percent(n,1) = 1.05;
    elseif noise_percent(n,1) < -1.05
        noise_percent(n,1) = -1.05;
    end
end
noisy = noise_percent - 1;
% PLOTTING HISTOGRAM PDF OF NOISE
% figure,
% histogram(noisy,'Normalization','pdf')
% xlabel('Noise')
% ylabel('pdf - f(noise)')
%plottingPDF; %plotting PDF of 5% noise
% ***************************
Xk_init = [0; 0];
Xinitial = 0;
Xk = [];
Xk_buffer = zeros(2,S_max+1);
Xk_buffer(:,1) = Xk_init;
Z_buffer = zeros(1,S_max+1);
P = eye(2);
I = eye(2);
H = [1 0];
Q = [1e-6 0; %1e-5^2  0; %1e-5^2;
	0 1e-6]; %0   1e-5^2];
W = [1e-5^2;
    1e-5^2];
dt = zeros(size(s));

% DETERMINE VEL LIMIT FROM INPUT FILE
% USING DATA FROM STS FILES
% pos=2;
% for i=1:S_max
%     v_lim(i,1)=vel_profile(pos-1,2)/3.6;
%     if v_lim(i,1)>=max_speed
%         v_lim(i,1)=max_speed;
%     end
%     if i*del_S>=fix(vel_profile(pos,1)*1000)
%         pos=pos+1;
%     end
% end

% DETERMINE GRADIENT PROFILE FROM INPUT FILE
% USING DATA FROM STS FILES
% pos=2;
% for i=1:(fix(1000/del_S*max(gradient(:,1))))
% %     grad_res(i,1)=gradient(pos-1,2)/((gradient(pos,1))-gradient(pos-1,1))*gravity;
%     grad_res(i,1)=gradient(pos-1,2)/1000*gravity;
%     if i*del_S>=fix(gradient(pos,1)*1000)
%         pos=pos+1;
%     end
% end
% grad_res(i+1,1)=grad_res(i,1);


% v_lim = v_permit;

% % CREATE TRAJECTORY OF SPEED REFERENCE
coasting = 1;
coasting_vel = 60;
for i = 1:del_S:S_max
    traction;
    error(i,1) = v_lim(i+1,1) - v(i,1);
    if error(i,1) > 1
        error(i,1) = 1;
    end
    
    if error(i,1) < -1
        error(i,1) = -1;
    end
    Fv(i,1) = Fv(i,1).*error(i,1);
    acc(i+1,1) = Fv(i,1)./mass;
    if acc(i+1,1) > max_accel
        acc(i+1,1) = max_accel;
    end
    resistances;
    forwardAccVel;
end
accel_F = accel;
backwardAccVel;
for i = 1:del_S:S_max
    if v(i,1) <= v_B(i,1)
        acceleration(i,1) = accel(i,1);
        v(i,1) = v(i,1);
    else
        acceleration(i-1,1) = -accel_B(i-1,1);
        v(i,1) = v_B(i,1);
    end
    dt(i+1,1) = 2*del_S/(v(i+1,1)+v(i,1));
end
acceleration(i,1) = -accel_B(i,1);
acceleration(i+1,1) = 0;
v_ref2 = v;
v(:,1) = 0;
error(:,1) = 0;
u(:,1) = 0;
acceleration(:,1) = 0;
accel(:,1) = 0;
acc(:,1) = 0;
Fv(:,1) = 0;
Bv(:,1) = 0;

for j = 1:del_S:S_max
    
    i = j;

% DEFINE ERROR, DELTA ERROR, SIGMA ERROR
    error(i,1) = v_ref2(i+1,1) - v(i,1);
    if i == 1
        del_e(i,1) = error(i,1) - 0;
    else
        del_e(i,1) = error(i,1) - error(i-1,1);
    end
    sig_e(i,1) = trapz(error);
% *************************************************************************


%     % if you want to test the system without the controller,
%     % uncomment lines below, comment pid_control, and change error to u
%     % *****************************************************
%     u_nolimit(i,1) = u(i,1);
%     if u(i,1) > 1
%         u(i,1) = 1;
%     end
%     
%     if u(i,1) < -1
%         u(i,1) = -1;
%     end
%    % *****************************************************
%     
% FUZZY ALPHA
    fuzzy_in(i,1) = error(i,1);
    fuzzy_in(i,2) = del_e(i,1);
    fuzzy_in(i,3) = v(i,1)/v_ref2(i,1)*100;
    alpha(1,1) = 0.5; %0.76;
    omega = 0.6; %0.35; %from 0.2 to 0.6;
    h(i,1) = evalfis(fuzzy_in(i,1:2),fuz_alpha);
    if alpha(i,1) > 0.5
        alpha(i+1,1) = alpha(i,1) + omega*h(i,1).*(1-alpha(i,1));
    elseif alpha(i,1) <= 0.5
        alpha(i+1,1) = alpha(i,1) + omega*h(i,1).*alpha(i,1);
    end
% 
%     %TRAJECTORY-BASED SWITCH ON Kp
% 	%traction and braking weights
        weight_T = 2+accel_F(i,1);
        weight_B = 1+accel_B(i,1);
    if accel_B(i,1) > 0
        kp = 1.2*alpha(i+1,1).*kcr*(weight_B); %*3.2; %*2;
    else
        kp = 1.2*alpha(i+1,1).*kcr*(weight_T); %*3;
    end
% %************************************ 
%     kp = 1.2*alpha(i+1,1).*kcr; % UNCOMMENT THIS LINE IF YOU WANT TO
% %     DEACTIVATE TRAJECTORY-BASED SWITCH AND COMMENT THE LINES 541-547
    ti = 0.75*pcr/(1+alpha(i+1,1));
    td = 0.25*ti;
% ********************************************************************
    
    pid_control;
    kp_buffer(i,1) = kp;
    ti_buffer(i,1) = ti;
    td_buffer(i,1) = td;
%     % applying control signal noise
%     randomNoise_u;
%     u_noNoise(i,1) = u(i,1);
%     u(i,1) = u(i,1) + u_noise(i+1,1);
%     Q = [u_noise(i+1,1) u_noise(i+1,1);
%         u_noise(i+1,1)  u_noise(i+1,1)];
%     W = [u_noise(i+1,1);
%         u_noise(i+1,1)];
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     applying process noise
%     u(i,1) = u(i,1) + 1e-6^2;
%     %%%%%%%%%%%%%%%%%%%%%%%%
    

    System_TrajecSwitch; % UNCOMMENT THIS LINE IF YOU WANNA USE
%     TRAJECTORY-BASED SWITCH
%     normalRunning; % UNCOMMENT THIS LINE IF YOU WANNA DEACTIVATE 
%     TRAJECTORY-BASED SWITCH
        
%     applying measurement errors
%     randomNoise;
%     defining known inputs (u) for Kalman filter
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     v_noNoise(i+1,1) = v(i+1,1); %/ noise(i+1,1);
%     v(i+1,1) = v(i+1,1) * noise_percent(i+1,1);
%     v_noise(i+1,1) = v(i+1,1);
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if i == 1
        dt(i+1,1) = 2*del_S/(v(i+1,1)+v(i,1)) + dwell;
    else
        dt(i+1,1) = 2*del_S/(v(i+1,1)+v(i,1));
    end
    j = i;
    k = i;
% using Kalman Filter
%         KalFil;
%         v(k+1,1) = Xk(1,1);
% ******************************************

% LIMIT SPEED OUTPUT IF IT EXCEEDS LOWER OR UPPER BOUND
    if v(k+1,1) > v_lim(k+1,1)
        v(k+1,1) = v_lim(k+1,1);
    end
    if v(k+1,1) <= 0
        v(k+1,1) = 0;
    end
    j = k;
% ******************************************

end

figure
plot(s, v,'b-');
hold on
plot(s, v_ref2,'g-.');
plot(s, v_lim,'m--');
plot(s, v_noise,'k:');
% plot(s, v_noNoise,'r--');
% plot(s,fuzz_out(:,1));
% plot(s,fuzz_out(:,2));
% plot(s,fuzz_out(:,3));
% plot(s,v_noise,'r:');
% plot(s, Xk_buffer(1,:),'r:');
grid on
xlabel('Distance (m)')
ylabel('Speed (m/s)')
title('Speed comparison')
legend('Actual speed','Speed reference','Speed limit','v noise');%,'v noNoise') %,'Kp','Ki','Kd')
% axis([0 100 0 12])

% figure,
% plot(s,fuzz_out(:,1));
% grid on;
% title('Kp')
% figure,
% plot(s,fuzz_out(:,2));
% grid on;
% title('Ki')
% figure,
% plot(s,fuzz_out(:,3));
% grid on;
% title('Kd')

% figure,
% plot(s,TrMinTref)
% grid on
% title('Percentage of run time diff')

dev = v_ref2 - v;
e_percent_pos = v_ref2./100*3;
over_pos = v_ref2./100*5;
e_percent_neg = -e_percent_pos;
over_neg = -over_pos;
figure,
plot(s./1000,dev.*3.6);
hold on;
plot(s./1000,e_percent_pos.*3.6);
plot(s./1000,e_percent_neg.*3.6);
% plot(s,over_pos)
% plot(s,over_neg)
grid on;
xlabel('Distance (km)')
ylabel('Speed (km/h)')
title('Error deviation between Vref and Vact')
legend('error deviation','+3% error limit','-3% error limit')
axis tight

timeJerk_j;
plotting;

% % plotting jerk
% figure
% plot(s, jerkj);
% grid on
% title('Jerk');
% xlabel('Distance (m)');
% ylabel('Jerk (m/s^3)');