function plotSetup(ax,plotList)
%% Lines
% This script change a plot in black and white style.
lineStyles = {'-','--','-.'};
markerStyles = {'none','none','none','s','p','x','d','.','*','^','v','<','>','h'};
rng(0,'twister');

for i = 1:length(plotList)
    plotHandle = plotList(i);
    % Set black
    set(plotHandle,'Color','k');
    
    % Random line style and marker
    %lineStyle = lineStyles{randi([1,length(lineStyles)])};
    %marker = markerStyles{randi([1,length(markerStyles)])};
    lineStyle = lineStyles{mod(i-1,length(lineStyles))+1};
    marker = markerStyles{mod(i-1,length(markerStyles))+1};
    
    % Set line and marker
    set(plotHandle, 'LineStyle', lineStyle);
    set(plotHandle, 'Marker', marker);
    
    % Set line and marker thickness
    set(plotHandle, 'LineWidth', 1);
    set(plotHandle, 'MarkerSize', 6);
end

%% Axes
% Grid
grid(ax,'on');
grid(ax,'minor');

% Color
set(ax, 'XColor', 'k');
set(ax, 'YColor', 'k');

% % Font
set(ax, 'FontSize', 20);