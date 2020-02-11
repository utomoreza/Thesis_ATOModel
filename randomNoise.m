%***********************************************************
% define 3% random noise
% randValue = rand;
% if randValue >= 0 && randValue <= 0.01
% 	noise(i,1) = 1.03;
% end
% 
% if randValue > 0.01 && randValue <= 0.08
%     noise(i,1) = 1.02;
% end
% 
% if randValue > 0.08 && randValue <= 0.25
% 	noise(i,1) = 1.01;
% end
% 
% if randValue > 0.25 && randValue <= 0.75
%     noise(i,1) = 1.0001;
% end
% 
% if randValue > 0.75 && randValue <= 0.92
%     noise(i,1) = 0.99;
% end
% 
% if randValue > 0.92 && randValue <= 0.99
%     noise(i,1) = 0.98;
% end
% 
% if randValue > 0.99 && randValue <= 1
%     noise(i,1) = 0.97;
% end
% 
% v(i,1) = v(i,1) * noise(i,1);

%***********************************************************
% define 5% random noise
randValue = rand;
if randValue >= 0 && randValue <= 0.01
	noise(i+1,1) = 1.02; %1.05;
end

if randValue > 0.01 && randValue <= 0.08
    noise(i+1,1) = 1.009; %1.03;
end

if randValue > 0.08 && randValue <= 0.25
	noise(i+1,1) = 1.005; %1.02;
end

if randValue > 0.25 && randValue <= 0.75
    noise(i+1,1) = 1.00000000000000;
end

if randValue > 0.75 && randValue <= 0.92
    noise(i+1,1) = 0.995; %0.98;
end

if randValue > 0.92 && randValue <= 0.99
    noise(i+1,1) = 0.991; %0.97;
end

if randValue > 0.99 && randValue <= 1
    noise(i+1,1) = 0.98; %0.95;
end

v(i+1,1) = v(i+1,1) * noise(i+1,1);
noise_pure(i+1,1) = noise(i+1,1) - 1;
if v(i+1,1) > v_lim(i+1,1)
    v(i+1,1) = v_lim(i,1);
end

%***********************************************************
% define 10% random noise
% randValue = rand;
% if randValue >= 0 && randValue <= 0.01
% 	noise(i,1) = 1.1;
% end
% 
% if randValue > 0.01 && randValue <= 0.08
%     noise(i,1) = 1.08;
% end
% 
% if randValue > 0.08 && randValue <= 0.25
% 	noise(i,1) = 1.03;
% end
% 
% if randValue > 0.25 && randValue <= 0.75
%     noise(i,1) = 1.02;
% end
% 
% if randValue > 0.75 && randValue <= 0.92
%     noise(i,1) = 0.94;
% end
% 
% if randValue > 0.92 && randValue <= 0.99
%     noise(i,1) = 0.98;
% end
% 
% if randValue > 0.99 && randValue <= 1
%     noise(i,1) = 0.9;
% end
% 
% v(i,1) = v(i,1) * noise(i,1);

%***********************************************************
% define 20% random noise
% randValue = rand;
% if randValue >= 0 && randValue <= 0.01
% 	noise(i,1) = 1.2;
% end
% 
% if randValue > 0.01 && randValue <= 0.08
%     noise(i,1) = 1.15;
% end
% 
% if randValue > 0.08 && randValue <= 0.25
% 	noise(i,1) = 1.1;
% end
% 
% if randValue > 0.25 && randValue <= 0.75
%     noise(i,1) = 1.05;
% end
% 
% if randValue > 0.75 && randValue <= 0.92
%     noise(i,1) = 0.9;
% end
% 
% if randValue > 0.92 && randValue <= 0.99
%     noise(i,1) = 0.95;
% end
% 
% if randValue > 0.99 && randValue <= 1
%     noise(i,1) = 0.8;
% end
% 
% v(i,1) = v(i,1) * noise(i,1);

%***********************************************************
% define 50% random noise
% randValue = rand;
% if randValue >= 0 && randValue <= 0.01
% 	noise(i,1) = 1.5;
% end
% 
% if randValue > 0.01 && randValue <= 0.08
%     noise(i,1) = 1.4;
% end
% 
% if randValue > 0.08 && randValue <= 0.25
% 	noise(i,1) = 1.2;
% end
% 
% if randValue > 0.25 && randValue <= 0.75
%     noise(i,1) = 1.1;
% end
% 
% if randValue > 0.75 && randValue <= 0.92
%     noise(i,1) = 0.8;
% end
% 
% if randValue > 0.92 && randValue <= 0.99
%     noise(i,1) = 0.6;
% end
% 
% if randValue > 0.99 && randValue <= 1
%     noise(i,1) = 0.5;
% end
% 
% v(i,1) = v(i,1) * noise(i,1);