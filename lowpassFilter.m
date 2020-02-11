% Lowpass filter

n = 500;
f = [0 0.07 0.1 1];
m = [1  1  0 0];
b = fir2(n,f,m);

%plot(x(1:length(b)),b);
v_lpf = abs(filter(b, 1, v));