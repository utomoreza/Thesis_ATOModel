% load StratfordLewisham
% data = table2array(StratfordLewisham);

load StratfordWoolwich %StratfordLewisham
data = table2array(StratfordWoolwich); %table2array(StratfordLewisham);


% DEFINE DISTANCES AND STATION STOPS FROM INPUT FILE
init = 0;
idx = 1;
refined = [];
j = 1;
NAN_ornot = isnan(data);
for i = 1:length(data)
    if NAN_ornot(i,1) == 1
        EMPTY_ornot = isempty(refined);
        if EMPTY_ornot == 0
            refined(1,idx:(init+data(i-1,1))) = init:1:(init+data(i-1,1)-1);
            init = refined(1,end) + 1;
            idx = init + 1;
        else
            refined = init:1:data(i-1,1);
            init = data(i-1,1) + 1;
            idx = init + 1;
        end
        stop(j,1) = init-1;
        j = j+1;
    end
end
refined = refined';
% figure,plot(refined)
% figure,plot(station)


% INPUT EACH STATION'S NAME
% IF YOU DON'T USE THE ROUTE OF STRATFORD INT'L - WOOLWICH ARSENAL
% COMMENT LINES BELOW
station = stop;
station(2:end+1,2) = station(1:end,1);
station(1,2) = 0;
station(1:end,1) = 0:11;
station_name = string({'Stratford International','Stratford', ...
    'Stratford High Street','Abbey Road','West Ham','Star Lane',...
    'Canning Town','West Silvertown','Pontonn Dock',...
    'London City Airport','King George V','Woolwich Arsenal'});
station_name = char(station_name);
% figure,plot(station(:,2)/1000,station(:,1))
% hold on
% plot(station(:,2)/1000,station(:,1),'ko')
% grid on
% for i = 2:length(station)-1
%     text(station(i,2)/1000+0.170,i-1,station_name(:,:,i));
% end
% text(station(1,2)/1000+0.200,1-0.7,station_name(:,:,1));
% text(station(12,2)/1000-3.150,12-1,station_name(:,:,12));
% yticks([0 1 2 3 4 5 6 7 8 9 10 11]);
% yticklabels({'1','2','3','4','5','6','7','8','9','10','11','12'});
% title('Position of Stations')
% xlabel('Distance (km)')
% ylabel('Order of stations')
% axis([0 12 0 11.5])


% DEFINE VELOCITY LIMIT FROM INPUT FILE
v_lim = zeros(size(refined));
idx = 1;
find_stat = find(NAN_ornot);
for i = 1:length(data)-1
    if data(i,3) ~= data(i+1,3)
        if i < find_stat(1,1)
            v_lim(idx:data(i,1)) = data(i,3);
            idx = data(i,1) + 1;
        else
            for j = 2:length(find_stat)
                if i >= find_stat(j-1) && i < find_stat(j)    
                    v_lim(idx:(data(i,1)+stop(j-1))) = data(i,3);
                    idx = data(i,1) + stop(j-1) + 1;
                end
            end
        end
    elseif i == length(data)-1
        v_lim(idx:(data(i,1)+stop(j-1))) = data(i,3);
        idx = data(i,1) + stop(j-1) + 1;
    end
end
v_lim(:) = v_lim(:)./3.6;
% figure, plot(v_lim)


% DETERMINE PERMITTED VELOCITY
v_permit = zeros(size(refined));
idx = 1;
find_stat = find(NAN_ornot);
for i = 1:length(data)-1
    dev_s = data(i+1,1) - data(i,1);
    if dev_s ~= 0
        if i < find_stat(1,1)
            v_permit(idx:data(i,1)) = data(i,4);
            idx = data(i,1) + 1;
        else
            for j = 2:length(find_stat)
                if i >= find_stat(j-1) && i < find_stat(j)
                    if NAN_ornot(i,1) == 1
                        v_permit(idx:(data(i-1,1)+stop(j-1))) = data(i,4);
                        idx = data(i-1,1) + stop(j-1) + 1;
                    else
                        v_permit(idx:(data(i,1)+stop(j-1))) = data(i,4);
                        idx = data(i,1) + stop(j-1) + 1;
                    end
                end
            end
        end
    elseif i == length(data)-1
        v_permit(idx:(data(i,1)+stop(j-1))) = data(i,4);
        idx = data(i,1) + stop(j-1) + 1;
    end
end
for i = 1:length(data)-1
    if data(i,4) ~= data(i+1,4)
        dev_s = data(i+1,1) - data(i,1);
        dev_s_ISNAN = isnan(dev_s);
        dev_vpermit = data(i+1,4) - data(i,4);
        equal = dev_vpermit/dev_s;
        if dev_s_ISNAN == 0 && dev_s ~= 1 && dev_vpermit ~= 0
            if i < find_stat(1,1)
                for k = 1:dev_s
                    v_permit(data(i,1)+k) = v_permit(data(i,1)+k-1) + ...
                        equal;
                end
            else
                for n = 2:length(find_stat)
                    if i >= find_stat(n-1) && i < find_stat(n)
                        for k = 1:dev_s
                            v_permit(data(i,1)+stop(n-1)+k) = ...
                                v_permit(data(i,1)+stop(n-1)+k-1) + equal;
                        end
                    end
                end
            end
        end
    end
end
v_permit(:) = v_permit(:)./3.6;
v_permit(1330) = 0;
v_permit(1770) = 0;
v_permit(2408) = 0;
v_permit(3000) = 0;
v_permit(3788) = 0;
v_permit(4577) = 0;
v_permit(6274) = 0;
v_permit(7052) = 0;
v_permit(8150:8151) = 0;
v_permit(9199) = 0;
% figure,plot(v_permit)


%DETERMINE GRADIENT PROFILE FROM INPUT FILE
grad = zeros(size(refined));
idx = 1;
find_stat = find(NAN_ornot);
for i = 1:length(data)-1
    if data(i,2) ~= data(i+1,2)
        if i < find_stat(1,1)
            grad(idx:data(i,1)) = data(i,2);
            idx = data(i,1) + 1;
        else
            for j = 2:length(find_stat)
                if i >= find_stat(j-1) && i < find_stat(j)
                    grad(idx:(data(i,1)+stop(j-1))) = data(i,2);
                    idx = data(i,1) + stop(j-1) + 1;
                end
            end
        end
    elseif i == length(data)-1
        grad(idx:(data(i,1)+stop(j-1))) = data(i,2);
        idx = data(i,1) + stop(j-1) + 1;
    end
end
grad(idx) = grad(idx-1);
grad(:,1) = (grad(:,1)-30)./100;
% figure, plot(grad)
% axis([-200 12000 -0.055 0.045])
% grid on
% title('Gradient (per 1 m) - Stratford Int''l to Woolwich Arsenal')
% xlabel('Distance (m)')
% ylabel('Gradient (%)')


% INITIATE GRADIENT RESISTANCE IN THE FORM OF ACCELERATION
grad_res = zeros(size(refined));
for i = 1:length(grad)
    grad_res(i) = grad(i)*gravity;
end
% figure, plot(grad_res)


s = refined;

%%
% DEFINE ROUTE FROM CANNING TOWN TO WEST SILVERTOWN ONLY
% range = refined(6274)-refined(4577);
% s = 0:1:range;
% s = s';
% v_lim_1 = zeros(size(s));
% v_lim_1 = v_lim(4577:6274);
% v_lim = v_lim_1;
% grad_1 = zeros(size(s));
% grad_1 = grad(4577:6274);
% grad = grad_1;
% grad_res = grad.*gravity;