result = zeros(100,1);
resultAvgTot = zeros(10,1);
for LupInLup = 1:10
    for lup = 1:100
        coba;
        if lup == 1
            result(lup,1) = result(lup,1) + Tr_Bsize20;
        else
            result(lup,1) = result(lup-1,1) + Tr_Bsize20;
        end
    end
    resultAvg = max(result)/100;
    
    if LupInLup == 1
        resultAvgTot(LupInLup,1) = resultAvg;
    else
        resultAvgTot(LupInLup,1) = resultAvg + resultAvgTot(LupInLup-1,1);
    end
end

resultAvgTot_avg = max(resultAvgTot)/10
%disp(resultAvg)