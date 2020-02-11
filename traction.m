if choose_trac == 0
    if v(i,1) <= 10
        Fv(i,1) = max_trac;
    end
    
    if v(i,1) > 10 && v(i,1) <= max_speed
        Fv(i,1) = max_trac - (1000*(v(i,1)-10));
    end
elseif choose_trac == 1
%     SpeedVsTraction;
    if v(i,1) <= 14.9
        if i ~= 1
            Fv(i,1) = Power/v(i,1);
        else
            Fv(i,1) = max_trac;
        end
        if Fv(i,1) > max_trac
            Fv(i,1) = max_trac;
        end
    elseif v(i,1) > 14.9
        Fv(i,1) = Power2/v(i,1)^4;
        if Fv(i,1) > max_trac2
            Fv(i,1) = max_trac2;
        end
    end
end

%figure(1);
%plot(s, Fv)


%del_S = max_speed/length(v_ref);
%v=del_S:del_S:max_speed;
%v(1,1) = 0;
%v = v';
%Res=zeros(size(v));
%Power=100*1000;
%trac = Power./v/mass;
%Davis=[2964 0.23 0.005 0];
%for i=1:length(v)
%    if trac(i,1)>max_accel
%            trac(i,1)=max_accel;
%    end
%    Res(i,1)=Davis(1)+ v(i,1)*Davis(2) + (v(i,1))^2*Davis(3);
%end
%
%Res=Res./mass;
%accel=trac-Res;
%
%figure(2);
%plot(v,trac*mass)
%hold on
%plot(v,Res*mass, 'r')
%plot(v, accel*mass, ' g')
%title('specific traction, resistance and acceleration curve')
%xlabel('velocity')
%ylabel('Tractive Effort')
%legend('specific traction', 'resistance', 'acceleration')