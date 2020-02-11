clear all
clc

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

if you want to test the system using step input,
uncomment 4 lines below then comment "load v_ref", "load v_ref_ori" and "load v_lim"
v_ref = zeros(5001,1);
v_ref(:,1) = 10;
v_lim = zeros(size(s));
v_lim(:,1) = v_ref(:,1) + 1;

if you want to use the system without any signal reference,
uncomment lines below
v_ref = zeros(5001,1);
v_ref(1:226,1) = 10;
v_ref(227:1001,1) = 15;
v_ref(1002:4350,1) = 20; %v_ref(1002:4176,1) = 20;
v_ref(4351:5000,1) = 10;
v_ref(end,1) = 0;
v_lim = zeros(size(s));
v_lim(:,1) = v_ref(:,1);

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

altitude = zeros(size(s));
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
del_e = zeros(size(s));
del_ej = zeros(size(s));
sig_e = zeros(size(s));
sig_ej = zeros(size(s));
u = zeros(size(s));
uj = zeros(size(s));
v_kf = zeros(size(s));
del_v = zeros(size(s));
accBv = zeros(size(s));
accFv = zeros(size(s));
accRv = zeros(size(s));
accRvj = zeros(size(s));
accel = zeros(size(s));
accelj = zeros(size(s));
del_T_ref = zeros(size(s));
Time_ref = zeros(size(s));
v_noNoise = zeros(size(s));
v_noise = zeros(size(s));

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
dwell = 5;

gradResistance;

backwardAccVel;

for j = 1:del_S:S_max
    
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
    
    if j == step(j,1)
        v(j,1) = vel(j-1,1);
        for i = j:del_S:S_max
            error(i,1) = v_ref(i+1,1) - v(i,1);
            pid_control;
            if u(i,1) > 0
                traction;
                Fv(i,1) = Fv(i,1)*u(i,1);
                acc(i,1) = Fv(i,1)./mass;
                if acc(i,1) > max_accel
                    acc(i,1) = max_accel;
                end
            else
                if u(i,1) < 0
                    braking;
                    Bv(i,1) = Bv(i,1)*u(i,1);
                    acc(i,1) = Bv(i,1)./mass;
                    if acc(i,1) < max_decel
                        acc(i,1) = max_decel;
                    end
                else
                    acc(i,1) = 0;
                end
            end
            resistances;
            forwardAccVel;
        end % end of generating running time
        
        timeJerk;
        maxTime = Time(end,1);
        maxTime_ref = Time_ref(end,1);
        
        if maxTime <= (maxTime_ref + (maxTime_ref/100*3)) && ...
                maxTime >= (maxTime_ref - (maxTime_ref/100*3))
            acceleration(j:S_max,1) = accel(j:S_max,1);
            vel(j:S_max,1) = v(j:S_max,1);
        else
            if maxTime > (maxTime_ref + (maxTime_ref/100*3))
                for i = j:del_S:S_max
                    GenerateFullPower;
                end
                
                for i = j:del_S:S_max
                    errorj(i,1) = v_ref2(i+1,1) - vel(i,1);
                    normal_FullPower;
                    if j > 999 && j <= 1200
                        v_ref(j+1,1) = v_ref(1000,1);
                        if vel(j+1,1) > v_ref(j,1)
                            vel(j+1,1) = v_ref(j,1);
                        end
                    end
        
                    if j > 2000 && j <= 2500
                        v_ref(j+1,1) = 18;
                        if vel(j+1,1) > v_ref(j,1)
                            vel(j+1,1) = v_ref(j,1);
                        end
                    end
    
                    if j >= 3500
                        if v_ref_ori(j,1) >= 17
                            v_ref(j+1,1) = 17;
                            if vel(j+1,1) > v(j,1)
                                vel(j+1,1) = v(j,1);
                            end
                        end
                    end
                end
                
                
            else
                acceleration(j:S_max,1) = accel(j:S_max,1);
                vel(j:S_max,1) = v(j:S_max,1);
            end
        end % end of comparing running time
    else
        if maxTime <= (maxTime_ref + (maxTime_ref/100*3)) && ...
                maxTime >= (maxTime_ref - (maxTime_ref/100*3))
            acceleration(j:S_max,1) = accel(j:S_max,1);
            vel(j:S_max,1) = v(j:S_max,1);
        else
            errorj(j,1) = v_ref(j+1,1) - vel(j,1);
            normal;
        end
    end % end of step 100 m

end % end of entire for


end

end

v(1:6,1) = v_noise(1:6,1);

*************************************************************
Looping for calculating backward velocity
backwardAccVel;

*************************************************************
Looping for calculating time and jerk
timeJerk_j;

applying lowpass filter
lowpassFilter;

plotting important data
plotting;

figure(5)
plot(s./1000, Jerk);
grid on

clear i
accel_kf = zeros(size(s));
for i = 1:del_S:S_max
   accel_kf(i+1,1) = ((v_kf(i+1,1)).^2 - (v_kf(i,1)).^2)./2*del_S;
end
figure, plot(s, accel_kf)

figure, plot(s, [v v_kf].*3.6)
grid on;
hold on, plot(s, v_ref.*3.6);
legend('without filter', 'with filter', 'ref');