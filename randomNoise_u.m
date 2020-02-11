% define 0.1% random u_noise

if i == ten(i,1)
    randValue = rand;
    if randValue >= 0 && randValue <= 0.15
        u_noise(i,1) = 1e-5^2; %0.99999;
    end
    
    if randValue > 0.15 && randValue <= 0.3
        u_noise(i,1) = 0; %0.999995;
    end
    
    if randValue > 0.3 && randValue <= 0.7
        u_noise(i,1) = -1*1e-5^2; %1.000000;
    end
    
%     if randValue_u > 0.7 && randValue_u <= 0.85
%         u_noise(i,1) = 1.00015;
%     end
%     
%     if randValue_u > 0.85 && randValue_u <= 1.00
%         u_noise(i,1) = 1.00001;
%     end
else
    u_noise(i,1) = u_noise(i-1,1);
end

u(i,1) = u(i,1) + u_noise(i,1);
% u_noise_pure(i,1) = u_noise(i,1) - 1;
u_noNoise(i,1) = u(i,1) - u_noise(i,1);
noisy_u(i,1) = u(i,1);

if u(i,1) > 1
    u(i,1) = 1;
end
if u(i,1) < -1
    u(i,1) = -1;
end