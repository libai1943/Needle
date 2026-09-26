function a = ReplanAudit(z, d, c, p)
    q = reshape(z(2:end), d.n, 9);
    dd = d;

    if nargin > 3
        dd.centers = reshape(p(10:end), 3, []);
    end
    x = ReplanReplay(z, dd, c.audit_substeps);
    mm = 1000 / d.scale;
    poly = clearance_segments(q(:, 1:3), dd) * mm;
    dense = clearance_segments(x(:, 1:3), dd) * mm;
    samples = inf(1, numel(d.radii));

    for j = 1:numel(d.radii)
        samples(j) = min(sqrt(sum((x(:, 1:3) - dd.centers(:, j)') .^ 2, 2))) - d.radii(j) - d.needle_radius;
    end
    lower = samples * mm - d.vmax * mm * z(1) / (2 * c.audit_substeps) - c.audit_padding_mm;
    body_path = q(:, 1:3);

    if isfield(d, 'executed_prefix')
        body_path = [d.executed_prefix; body_path];
    end
    body = ReplanSelfCheck(body_path, d, c);
    a = struct('polyline_by_obstacle_mm', poly, 'replay_by_obstacle_mm', dense, 'clearance_lower_by_obstacle_mm', lower, 'polyline_clearance_mm', min(poly), 'replay_clearance_mm', min(dense), 'clearance_lower_mm', min(lower), 'terminal_mm', norm(x(end, 1:3)' - d.goal) * mm, 'self', body, 'position_mm', x(:, 1:3) * mm, 'substeps', c.audit_substeps);
    a.replay_bounds_ok = all(all(x(:, 1:3) >= d.lb' - 1e-8 & x(:, 1:3) <= d.ub' + 1e-8)) && min(x(:, 4)) >= -1e-8 && max(x(:, 4)) <= d.vmax + 1e-8 && all(cos(x(:, 6)) .^ 2 >= d.cos_limit ^ 2 - 1e-8);
    a.obstacle_ok = min([poly, dense, lower]) >= 0;
    a.feasible = a.obstacle_ok && body.feasible && a.terminal_mm <= c.audit_target_mm && a.replay_bounds_ok;
end

function answ = clearance_segments(x, d)
    a = x(1:end - 1, :);
    v = diff(x);
    den = max(sum(v .^ 2, 2), eps);
    answ = inf(1, numel(d.radii));
    for j = 1:numel(d.radii)
        t = max(0, min(1, sum((d.centers(:, j)' - a) .* v, 2) ./ den));
        dist = sqrt(sum((a + t .* v - d.centers(:, j)') .^ 2, 2));
        answ(j) = min(dist) - d.radii(j) - d.needle_radius;
    end
end
