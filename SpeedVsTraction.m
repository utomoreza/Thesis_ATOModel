% PLOTTING TRACTION VS SPEED
% FOR DISPLAY PURPOSE ONLY
kec = 0:0.1:max_speed;
kec = kec';
F1 = zeros(size(kec));
F1 = Power./kec;
F2 = Power2./(kec.^4);
Res=zeros(size(kec));
for i = 1:length(kec)
    if F1(i,1) > max_trac
        F1(i,1) = max_trac;
    end
    if F2(i,1) > max_trac2
        F2(i,1) = max_trac2;
    end
    Res(i,1)= 65 + kec(i,1)*35 + (kec(i,1))^2*12.43;
end
indeks = find(F1<max_trac2,1,'first');
F1(indeks(1):end) = F2(indeks(1):end);
figure,plot(kec.*3.6,[F1./1000 Res./1000])
title('Speed versus Traction & Running Resistance')
xlabel('Speed (km/h)')
ylabel('Traction/Resistance (kN)')
legend('Tractive effort','Running resistance')
grid on