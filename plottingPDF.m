rng(0,'twister');
mu = 0; %0 mean
std_dev = 0.015; %standard deviation 0.015 to make 5% Gaussian noise
y = std_dev.*randn(size(s)) + mu; %generate normal distribution

figure,
histogram(y,'Normalization','pdf')
xlabel('Noise')
ylabel('pdf - f(noise)')
% title('PDF of measurement noise - mean 0 and std dev 0.015');
grid on
axis([-0.05 0.05 0 28])
ax1=gca;
ax2 = axes('Position', get(ax1, 'Position'),'Color', 'none');
set(ax2, 'XAxisLocation', 'top','YAxisLocation','Right');
% set the same Limits and Ticks on ax2 as on ax1;
set(ax2, 'XLim', get(ax1, 'XLim'),'YLim', get(ax1, 'YLim'));
set(ax2, 'XTick', get(ax1, 'XTick'), 'YTick', get(ax1, 'YTick'));
OppTickLabelsX = {'95%' '96%' '97%' '98%' '99%' '100%' '101%' '102%' '103%' '104%' '105%'};
OppTickLabelsY = {'','','','','',''};
% Set the x-tick and y-tick  labels for the second axes
set(ax2, 'XTickLabel', OppTickLabelsX,'YTickLabel',OppTickLabelsY);
% set(get(ax1,'title'),'Position',[0 30 1.00011]);