u = 0:1;
u = u';
v(:,1) = 0;
% u(:,1) = 0.041;
for i = 1:length(u)
    normalRunning;
end

figure,plot(u,v(1:length(u)));
grid on;
figure,plot(s,v); grid on