% clear all;clc
x=[0:.01:16];
y=sin(3*x);
figure;hold all
Dx=50;y1=-1.2;y2=1.2;
for n=1:1:numel(x)
      plot(x,y);
      axis([x(n) x(n+Dx) y1 y2]);
      drawnow
end