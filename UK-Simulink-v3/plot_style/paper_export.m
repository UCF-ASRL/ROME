function paper_export(fig, pdfpath)
%PAPER_EXPORT  Write a figure as vector PDF plus a 300 dpi PNG twin.
%
%   PAPER_EXPORT(fig, 'figures/fig_name.pdf') writes fig_name.pdf (vector,
%   tight crop, white background) and fig_name.png (300 dpi, the resolution
%   Frontiers and most journals require for raster submission).
%
%   Every colour is forced explicit first, because MATLAB's dark UI theme can
%   leak grey axes and black legend boxes into a batch-mode export. Then
%   PAPER_CHECK runs: a legend or text label that covers data points is
%   reported on the console, every time, so it cannot go unnoticed.

try, theme(fig, 'light'); catch, end
set(fig, 'Color', 'w', 'InvertHardcopy', 'on');
for ax = findobj(fig, 'Type', 'axes').'
    set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', 'GridColor', [0.15 0.15 0.15]);
    set(get(ax, 'Title'),  'Color', 'k');
    set(get(ax, 'XLabel'), 'Color', 'k'); set(get(ax, 'YLabel'), 'Color', 'k'); set(get(ax, 'ZLabel'), 'Color', 'k');
end
for lg = findobj(fig, 'Type', 'legend').'
    set(lg, 'Color', 'w', 'TextColor', 'k');
end
drawnow;
n_over = paper_check(fig);
if n_over > 0
    fprintf('paper_export: %d overlap(s) flagged in %s\n', n_over, pdfpath);
end
pngpath = regexprep(pdfpath, '\.pdf$', '.png');
exportgraphics(fig, pdfpath, 'ContentType', 'vector', 'BackgroundColor', 'white');
exportgraphics(fig, pngpath, 'Resolution', 300,     'BackgroundColor', 'white');
end
