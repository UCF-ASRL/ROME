function fig = paper_figure(size, height_in)
%PAPER_FIGURE  Open a figure at the size it will be printed.
%
%   fig = PAPER_FIGURE('full')          7.00 in wide (a Frontiers two-column figure, 180 mm)
%   fig = PAPER_FIGURE('half')          3.36 in wide (two panels side by side at 0.48\linewidth)
%   fig = PAPER_FIGURE('third')         2.24 in wide (three panels side by side at 0.32\linewidth)
%   fig = PAPER_FIGURE('column')        3.35 in wide (a Frontiers one-column figure, 85 mm)
%   fig = PAPER_FIGURE([w h])           w by h inches
%   fig = PAPER_FIGURE(name, h)         named width with height h inches
%
%   Drawing at the printed size is what keeps 8 pt text at 8 pt on the page;
%   a figure drawn at 12 in and shrunk to 7 in prints its 9 pt labels at 5 pt,
%   below the 8 pt minimum most journals set. Heights default to 0.75 of the
%   width for 'third' and 'half', 0.55 for 'full'.

if ischar(size) || isstring(size)
    switch lower(char(size))
        case 'full',   w = 7.00; h = 0.55 * w;
        case 'half',   w = 3.36; h = 0.75 * w;
        case 'third',  w = 2.24; h = 0.90 * w;
        case 'column', w = 3.35; h = 0.75 * w;
        otherwise, error('paper_figure:size', 'unknown size name %s', size);
    end
    if nargin > 1 && ~isempty(height_in), h = height_in; end
else
    w = size(1); h = size(2);
end
fig = figure('Units', 'inches', 'Position', [1 1 w h], 'Color', 'w', 'Visible', 'off');
end
