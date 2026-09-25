function r = ResampleRoute(x, n)
    arc = [0; cumsum(sqrt(sum(diff(x) .^ 2, 2)))];
    [arc, idx] = unique(arc, 'stable');
    x = x(idx, :);

    if numel(arc) == 1
        r = repmat(x, n, 1);
    else
        r = interp1(arc, x, linspace(0, arc(end), n));
    end
end
