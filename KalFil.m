% B = [dt(k+1,1)    0;
%     0   dt(k+1,1)];
B = [1 0; %error(k+1,1)   error(k+1,1);
    0 1]; %error(k+1,1)    error(k+1,1)];
C = [1  1;
    1   1];
% if error(k,1) == 0 %&& s(k,1) < S_max-50
%     U = [error(k,1) + Q(1,1); %+ 1e-6^2;
%         0];
% else
    U = [error(k,1);
        0];
% end
A = [1  dt(k+1,1);
    0  0];
% if s(k,1) >= S_max-50 && s(k,1) < S_max-30
%     Q(:,:) = 1e-10 * 1e1;
%     R = 1e-3 * 1e-1;
% elseif s(k,1) >= S_max-30 && s(k,1) < S_max-20
%     Q(:,:) = 1e-10 * 1e2;
%     R = 1e-3 * 1e-2;
% elseif s(k,1) >= S_max-20 && s(k,1) < S_max-15
%     Q(:,:) = 1e-10 * 1e3;
%     R = 1e-3 * 1e-3;
% elseif s(k,1) >= S_max-15 && s(k,1) < S_max-10
%     Q(:,:) = 1e-10 * 1e4;
%     R = 1e-3 * 1e-4;
% elseif s(k,1) >= S_max-10 && s(k,1) < S_max-7
%     Q(:,:) = 1e-10 * 1e4;
%     R = 1e-3 * 1e-4;
% elseif s(k,1) >= S_max-7 && s(k,1) < S_max-5
%     Q(:,:) = 1e-10 * 1e4;
%     R = 1e-3 * 1e-4;
% elseif s(k,1) == S_max-5
%     Q(:,:) = 1e-10 * 1e4;
%     R = 1e-3 * 1e-4;
% elseif s(k,1) == S_max-4
%     Q(:,:) = 1e-10 * 1e5;
%     R = 1e-3 * 1e-4;
% elseif s(k,1) == S_max-3
%     Q(:,:) = 1e-10 * 1e5;
%     R = 1e-3 * 1e-4;
% elseif s(k,1) == S_max-2
%     Q(:,:) = 1e-10 * 1e5;
%     R = 1e-3 * 1e-5;
% elseif s(k,1) == S_max-1
%     Q(:,:) = 1e-10 * 1e5;
%     R = 1e-3 * 1e-5;
% else
    R = cov(noisy(1:k+1,1)); %0.008; %cov(v_noNoise(1:k+1,1)); %noisy(k+1,1);
% end

Xk_prev = A*Xk_init + B*U;% + B*W;

% Z is the measurement vector. In our
% case, Z = TrueData + RandomGaussianNoise
Z = v(k+1,1); %Vtrue(k+1) + R;
Z_buffer(k+1) = Z;

% Kalman iteration
P1 = A*P*A' + Q;
S = H*P1*H' + R;

% K is Kalman gain. If K is large, more weight goes to the measurement.
% If K is low, more weight goes to the model prediction.
K = P1*H'*inv(S);
if K(1,1) > 1
    K(1,1) = 1;
else
    if K(1,1) < 0
        K(1,1) = 0;
    end
end
if K(2,1) > 1
    K(2,1) = 1;
else
    if K(2,1) < 0
        K(2,1) = 0;
    end
end
K_buffer(:,k+1) = K;
P = (I - K*H)*P1; %P1 - K*H*P1;

Xk = Xk_prev + K*(Z-H*Xk_prev);
if k == S_max
    Xk(:,1) = 0;
end
Xk_buffer(:,k+1) = Xk;

% For the next iteration
Xk_init = Xk;