h = 1e-4;       % step size
X = -pi:h:pi;    % domain
f = sin(X);      % range
Y = diff(f);   % first derivative
Z = diff(Y);   % second derivative
figure,plot(X(:,1:length(Y)),Y,'r',X,f,'b', X(:,1:length(Z)),Z,'k')

satu = diff(v)'./del_Tj;

satu = zeros(size(s));
dua = zeros(size(s));
for i = 1:S_max
        satu(i,1) = (v_ref_ori(i+1,1) - v_ref_ori(i,1)) / del_T_ref(i+1,1);
end
for i = 1:S_max
    dua(i,1) = (satu(i+1,1) - satu(i,1)) / del_T_ref(i+1,1);
end

figure,plot(satu); grid on; title('acc_ref')
figure,plot(dua); grid on; title('jerk_ref')

figure,plot(accel); grid on
figure,plot(jerkj); grid on; title('real jerk')

v(t)

figure,plot(X)
diff(v);

figure,plot(del_Tj)
