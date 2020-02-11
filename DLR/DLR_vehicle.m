% DOCKLANDS LIGHT RAILWAY
% CLASS B2007
% @Copyright Transport for London

mass_trainset = 38200; % kg - tare mass each trainset (each trainset consists of 2 cars)
full_mass = 57300; % kg - mass each trainset in maximum capacity
n_train = 2; % or 3 - number of trainsets usually coupled
mass = n_train*mass_trainset; % kg - or n_train*full_mass; - total mass
lambda = 0.1;                    %0
inertial_mass = mass*(1+lambda);

Davis=[12.43 35 65];  % Davis equations - Cv^2 + Bv + A

Power = n_train*2*130000; % Watt - total power produced %used at v<=14.9 m/s
Power2 = 3960*Power; % obtained from trial-error-observation %used at v>14.9 m/s 
max_trac = 65000; %n_train*2*1200; % Nm - total traction produced %used at v<=14.9 m/s
max_trac2 = max_trac/2; %used at v>14.9 m/s

max_accel = 1.4; % m/s^2 - in-operation and maximum acc
max_speed = 80/3.6; % top speed 80 km/h

max_brake = 206300; % N - maximum braking force (emergency brake)
emerg_decel = -1.44; % m/s^2 - maximum deceleration (emergency brake)
max_decel = -1.44; % -0.8 m/s^2 - in-operation deceleration