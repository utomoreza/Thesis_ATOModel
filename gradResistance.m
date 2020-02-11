pos = 2;
grad_prof = zeros(size(s));
for i = 1:(fix(1000/del_S*max(gradient(:,1))))
    grad_prof(i,1) = gradient(pos-1,2)/1000*gravity;
    if i*del_S >= fix(gradient(pos,1)*1000)
        pos = pos+1;
    end
end
grad_prof(i+1,1) = grad_prof(i,1);

% gradient profile down to up
grad_res(route,1) = gradient(route,1)*mass*gravity;

% gradient profile up to down
grad_res(route,1) = gradient(route,2)*mass*gravity;

%integral of the gradient profile:
pos = 2;
for i = 1:fix(1000/del_S*max(gradient(:,1)))
   grad_1(i,1) = gradient(pos-1,2);
    if i*del_S >= fix(gradient(pos,1)*1000)
        pos=pos+1;
    end
end
altitude = cumtrapz(s(:,1), grad_1(:,1))/1000;
% plot(s/1000,altitude, 'k')
% xlabel('distance (km)')
% ylabel('altitude (m)')