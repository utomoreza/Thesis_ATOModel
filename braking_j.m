if vel(j,1) <= 15
    Bvj(j,1) = max_brake;
end

if vel(j,1) > 15
    Bvj(j,1) = max_brake - (1000*(vel(j,1)-15));
end