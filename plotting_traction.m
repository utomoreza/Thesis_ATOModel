v=1:1:max_speed;
v=v./3.6;
trac=Power./(v)/(Mass*1000);
figure, plot(v*3.6,trac.*1000*Mass)
grid on

kec = 1:0.1:max_speed;
kec = kec';
a_trac = Power./kec./mass;
Power_brake = max_brake.*kec;
braking = Power_brake./kec;
Res = zeros(size(kec));
for i = 1:length(kec)
    if a_trac(i,1) > max_accel
        a_trac(i,1) = max_accel;
    end
    Res(i,1) = Davis(3) + kec(i,1).*Davis(2) + (kec(i,1).^2).*Davis(1);
end
Res = Res./mass;
Accel = a_trac-Res;

figure, plot(kec*3.6,a_trac.*mass)
hold on
plot(kec*3.6,Res.*mass)
plot(kec*3.6,Accel)
grid on
legend('Traction','Running Resistance','Acceleration')

kec = 1:0.1:80;
kec = kec';
d_trac = Power./kec./mass;
Res = zeros(size(kec));
for i = 1:length(kec)
    if d_trac(i,1) > max_accel
        d_trac(i,1) = max_accel;
    end
end