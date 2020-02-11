accel(i,1) = acc(i+1,1) - accRtot(i+1,1);
if i == S_max
    accel(end,1) = 0;
end

%if del_v(i,1) >= 0
%    if Fv(i,1) == (Rv(i,1) + grad_res(i,1))
%        acc(i+1,1) = 0;
%    else
%        accFv(i,1) = Fv(i,1)./mass;
%        if accFv(i,1) > max_accel
%            accFv(i,1) = max_accel;
%        end
%        accRv(i,1) = Rv(i,1)./mass;
%        acc(i+1,1) = accFv(i,1) - accRv(i,1) - grad_res(i,1);
%    end
%else
%    %if del_v(i,1) < 0
%	accBv(i,1) = Bv(i,1)./mass;
%	if accBv(i,1) < max_decel
%        accBv(i,1) = max_decel;
%    end
%	accRv(i,1) = Rv(i,1)./mass;
%	acc(i+1,1) = accBv(i,1) - accRv(i,1) - grad_res(i,1);        
    %else
        %acc(i+1,1) = 0;
    %end
%end

if v(i,1) < v_lim(i+1,1)     %compares current vel to next limit
    v(i+1,1)=abs((v(i,1).^2+2*accel(i,1)*(del_S)).^0.5);
    if v(i+1,1) > v_lim(i+1,1)
        v(i+1,1) = v_lim(i+1,1);
    end
    
else
    if accel(i,1) < 0
        v(i+1,1) = abs((v(i,1).^2+2*(accel(i,1))*(del_S)).^0.5);
        if v(i+1,1) > v_lim(i+1,1)
            v(i+1,1) = v_lim(i+1,1);
        end
    else
        accel(i,1) = 0;          % zero acceleration
        if v_lim(i+1,1) == 0    % if vel_limit = 0
            v(i+1,1) = 0;      % then vel = 0
        else                    % otherwise velF is the same as previous
            v(i+1,1) = v_lim(i,1);
        end
    end
end

% if i == S_max
%     v(end,1) = 0;
% end