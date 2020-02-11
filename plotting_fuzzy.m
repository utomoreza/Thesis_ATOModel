oi = readfis('Fuzzy-Adaptation');
figure,
plotfis(oi);
title('Fuzzy adaptation block diagram')

figure,
plotmf(oi,'input',1);
title('MFs of E')

figure,
plotmf(oi,'input',2);
title('MFs of R')

figure,
plotmf(oi,'output',1);
title('MFs of H')
axis([-1 2 0 1.1])