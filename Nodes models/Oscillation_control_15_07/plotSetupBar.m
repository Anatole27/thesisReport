function plotSetup
%% Scale

hbar = findobj(gcf, 'type', 'bar');
ax = gca;
b = max([hbar(1).YData,hbar(2).YData])
ylim([0 b*1.2]);

%% Lines
% This script change a plot in black and white style.
for i = 1:length(hbar);
    plotHandle = hbar(i)
    % Set black
    plotHandle.EdgeColor = 'k';
    plotHandle.FaceColor = [0.5 0.5 0.5];
end


%% Axes
% Grid
grid(ax,'on');
grid(ax,'minor');

% Font
ax.FontSize = 20;
ax.FontName = 'LM Roman 12'
ax.Title.FontName = 'LM Roman 12'
ax.Title.FontSize = 20;
ax.XLabel.FontSize = 20;
ax.XLabel.FontName = 'LM Roman 12';
ax.YLabel.FontSize = 20;
ax.YLabel.FontName = 'LM Roman 12';