function n = paper_check(fig)
%PAPER_CHECK  Warn when a legend or a text label sits on top of plotted data.
%
%   n = PAPER_CHECK(fig) returns the number of overlaps found and prints one
%   warning per legend or text object whose box contains data points of a
%   line in the same 2-D axes. PAPER_EXPORT calls it on every export, so a
%   figure that covers its own data is flagged the moment it is written.
%   Three-dimensional axes are skipped (text extents are not defined there).

n = 0;
for ax = findobj(fig, 'Type', 'axes').'
    try
        v = get(ax, 'View');
        if any(abs(v - [0 90]) > 1e-6), continue; end          % 3-D: skip
        lines = findobj(ax, 'Type', 'line');
        X = []; Y = [];
        for h = lines.'
            x = get(h, 'XData'); y = get(h, 'YData');
            if numel(x) < 2, continue; end                     % markers alone
            X = [X; x(:)]; Y = [Y; y(:)];                     %#ok<AGROW>
        end
        if isempty(X), continue; end
        islogy = strcmp(get(ax, 'YScale'), 'log'); islogx = strcmp(get(ax, 'XScale'), 'log');
        fy = @(y) y; if islogy, fy = @(y) log10(max(y, realmin)); end
        fx = @(x) x; if islogx, fx = @(x) log10(max(x, realmin)); end
        Xs = fx(X); Ys = fy(Y);
        boxes = {};
        lg = get(ax, 'Legend');
        if ~isempty(lg) && strcmp(get(lg, 'Visible'), 'on')
            ap = get(ax, 'Position'); lp = get(lg, 'Position');   % normalized figure units
            xl = fx(xlim(ax)); yl = fy(ylim(ax));
            bx = xl(1) + (lp(1) - ap(1)) / ap(3) * diff(xl);
            by = yl(1) + (lp(2) - ap(2)) / ap(4) * diff(yl);
            bw = lp(3) / ap(3) * diff(xl); bh = lp(4) / ap(4) * diff(yl);
            boxes{end+1} = {'legend', [bx by bw bh]};           %#ok<AGROW>
        end
        for t = findobj(ax, 'Type', 'text').'
            s = get(t, 'String'); if iscell(s), s = strjoin(s, ' '); end
            if isempty(strtrim(s)), continue; end
            e = get(t, 'Extent');                                % data units
            e = [fx(e(1)), fy(e(2)), fx(e(1)+e(3)) - fx(e(1)), fy(e(2)+e(4)) - fy(e(2))];
            boxes{end+1} = {sprintf('text "%s"', s), e};         %#ok<AGROW>
        end
        ttl = get(get(ax, 'Title'), 'String'); if iscell(ttl), ttl = strjoin(ttl, ' '); end
        for k = 1:numel(boxes)
            b = boxes{k}{2};
            in = Xs >= b(1) & Xs <= b(1) + b(3) & Ys >= b(2) & Ys <= b(2) + b(4);
            if any(in)
                warning('paper_check:overlap', '%s covers %d data points in panel "%s"', ...
                        boxes{k}{1}, nnz(in), ttl);
                n = n + 1;
            end
        end
    catch err
        warning('paper_check:skipped', 'axes skipped: %s', err.message);
    end
end
end
