function b = ReplanStopState(start, d, c)
    b = struct('start', start, 'stopped', start, 'duration', 0, 'position_mm', [], 'clearance_lower_mm', -Inf, 'feasible', false);

    if numel(start) ~= 9 || any(~isfinite(start)) || start(4) < 0 || start(4) > d.vmax || cos(start(8)) ^ 2 < d.cos_limit ^ 2
        return
    end
    T = max(0, start(4) / d.amax);
    n = max(2, ceil(T / .005) + 1);
    dt = T / (n - 1);
    x = start([1:4, 7:9]);
    x = x(:);

    if T == 0
        path = x';
    else
        dd = d;
        dd.n = n;
        q = zeros(n, 9);
        q(1, :) = start';
        q(:, 5) = -d.amax;
        q(:, 6) = 0;
        path = ReplanReplay([dt; q(:)], dd, 1);
    end
    stopped = start;
    stopped([1:4, 7:9]) = path(end, :)';
    stopped(4:6) = 0;
    clearance = Inf;

    for j = 1:numel(d.radii)
        clearance = min(clearance, min(sqrt(sum((path(:, 1:3) - d.centers(:, j)') .^ 2, 2))) - d.radii(j) - d.needle_radius);
    end
    lower = clearance * 1000 / d.scale - d.vmax * 1000 / d.scale * dt / 2 - c.audit_padding_mm;
    bounds = all(all(path(:, 1:3) >= d.lb' - 1e-8 & path(:, 1:3) <= d.ub' + 1e-8)) && all(cos(path(:, 6)) .^ 2 >= d.cos_limit ^ 2 - 1e-8) && all(all(abs(path(:, 5:7)) <= 20 + 1e-8));
    body = path(:, 1:3);

    if isfield(d, 'executed_prefix')
        body = [d.executed_prefix; body];
    end
    body_check = ReplanSelfCheck(body, d, c);
    b = struct('body_check', body_check, 'state_path', path, 'start', start, 'stopped', stopped, 'duration', T, 'position_mm', path(:, 1:3) * 1000 / d.scale, 'clearance_lower_mm', lower, 'feasible', lower >= 0 && bounds && body_check.feasible);
end
