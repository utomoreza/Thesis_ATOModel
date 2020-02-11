% Dwelling delay
i = del_S;
del_T(i+1,1) = 2*del_S/(v(i+1,1)+v(i,1)) + dwell;
Time(i+1,1) = Time(i,1)+del_T(i+1,1);

% Running time delay
if i > 999 && i <= 1200
	v_ref(i+1,1) = v_ref(1000,1);
end

if i > 2000 && i <= 2500
	v_ref(i+1,1) = 18;
end

if i >= 3500
	if v_ref_ori(i,1) >= 17
        v_ref(i+1,1) = 17;
	end
end

% Emergency situations
