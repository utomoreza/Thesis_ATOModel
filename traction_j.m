if vel(j,1) <= 10
    Fvj(j,1) = max_trac;
end

if vel(j,1) > 10 && vel(j,1) <= 22.2
    Fvj(j,1) = max_trac - (1000*(vel(j,1)-10));
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