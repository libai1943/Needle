function v = CheckTrajectory(z, d, c, mode, extra)
    if nargin < 5
        extra = struct();
    end
    if strcmp(mode, 'posterior')
        v = PosteriorAudit(z, d, c);
    else
        v = NodalCheck(z, d, c, mode, extra);
    end
end

function v = NodalCheck(z, d, c, mode, extra)
    p = [zeros(9, 1); d.centers(:)];
    v = struct('feasible', false, 'eq', Inf, 'ineq', Inf, 'terminal_mm', Inf, 'clearance_mm', -Inf, 'segment_clearance_mm', NaN, 'duration', Inf, 'length_mm', Inf, 'effort', Inf, 'smoothness', Inf, 'cost', Inf);
    n = d.n;

    if numel(z) ~= 1 + 9 * n || any(~isfinite(z)) || z(1) <= 0
        return
    end
    h = z(1);
    q = reshape(z(2:end), n, 9);
    ids = [1:4, 7:9];
    res = q(2:end, ids) - NeedleStep(q(1:end - 1, ids), q(1:end - 1, 5:6), h, d);
    e = reshape(res(:, [1:3, 5, 6, 4, 7]), [], 1);
    qs = [.1 * d.scale * ones(1, 3), d.vmax, d.amax, d.wmax, 1, 1, 1];
    es = kron([.1 * d.scale * ones(3, 1); 1; 1; d.vmax; 1], ones(n - 1, 1));
    low = [d.lb', 0, -d.amax, -d.wmax, -20, -20, -20];
    high = [d.ub', d.vmax, d.amax, d.wmax, 20, 20, 20];
    bound = max([0; reshape((low - q) ./ qs, [], 1); reshape((q - high) ./ qs, [], 1); (d.tmin - (n - 1) * h) / d.tmax; ((n - 1) * h - d.tmax) / d.tmax]);
    boundary = [(q(1, 3) - d.start(3)) / qs(3), q(1, 4:6) ./ qs(4:6)];
    bound = max([bound, (abs(q(1, 1:2) - d.start(1:2)') - d.start_slack) ./ qs(1:2), -q(1, 8), q(1, 8) - acos(d.cos_limit), abs(q(1, 7)) - pi, abs(q(1, 9)) - pi]);
    boundary = [boundary, q(end, 4:6) ./ qs(4:6)];
    v.terminal_mm = norm(q(end, 1:3) - d.goal') * 1000 / d.scale;

    if ~strcmp(mode, 'presolve')
        boundary = [boundary, (q(end, 1:3) - d.goal') ./ qs(1:3)];
    end
    v.eq = max(abs([e ./ es; boundary(:)]));
    centers = reshape(p(10:end), 3, []);
    clearance = inf(n, 1);
    sphere_violation = 0;

    for j = 1:size(centers, 2)
        radius = d.radii(j) + d.needle_radius;
        dist = sqrt(sum((q(:, 1:3) - centers(:, j)') .^ 2, 2));
        clearance = min(clearance, dist - radius);
        sphere_violation = max(sphere_violation, max((radius ^ 2 - dist .^ 2) / max(radius ^ 2, (.001 * d.scale) ^ 2)));
    end
    if isfield(extra, 'corridor')
        bound = max([bound; reshape((extra.corridor(:, 1:3) - q(:, 1:3)) / (.1 * d.scale), [], 1); reshape((q(:, 1:3) - extra.corridor(:, 4:6)) / (.1 * d.scale), [], 1)]);
    end
    v.ineq = max([bound; sphere_violation; d.cos_limit ^ 2 - cos(q(:, 8)) .^ 2]);
    v.clearance_mm = min(clearance) * 1000 / d.scale;
    v.duration = (n - 1) * h;
    v.length_mm = sum(sqrt(sum(diff(q(:, 1:3)) .^ 2, 2))) * 1000 / d.scale;
    v.effort = h * sum((q(:, 5) / d.amax) .^ 2 + (q(:, 6) / d.wmax) .^ 2);
    v.smoothness = sum(diff(q(:, 5) / d.amax) .^ 2 + diff(q(:, 6) / d.wmax) .^ 2) / h;
    v.cost = v.duration + c.effort_weight * v.effort + c.smooth_weight * v.smoothness;

    if isfield(c, 'attitude_weight') && c.attitude_weight > 0
        gap = cos(q(:, 8)) .^ 2 - d.cos_limit ^ 2;
        positive_gap = .5 * (gap + sqrt(gap .^ 2 + 1e-12));
        v.cost = v.cost - c.attitude_weight * h * sum(log(positive_gap / (1 - d.cos_limit ^ 2)));
    end
    v.feasible = v.eq <= c.eq_tol && v.ineq <= c.ineq_tol;
end

function a = PosteriorAudit(z, d, c)
    q = reshape(z(2:end), d.n, 9);
    dd = d;
    x = Replay(z, dd, c.audit_substeps);
    mm = 1000 / d.scale;
    poly = clearance_segments(q(:, 1:3), dd) * mm;
    dense = clearance_segments(x(:, 1:3), dd) * mm;
    samples = inf(1, numel(d.radii));

    for j = 1:numel(d.radii)
        samples(j) = min(sqrt(sum((x(:, 1:3) - dd.centers(:, j)') .^ 2, 2))) - d.radii(j) - d.needle_radius;
    end
    lower = samples * mm - d.vmax * mm * z(1) / (2 * c.audit_substeps) - c.audit_padding_mm;
    body_path = q(:, 1:3);
    body = SelfCheck(body_path, d, c);
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

function x = Replay(z, d, substeps)
    persistent cache

    if isempty(cache)
        cache = containers.Map();
    end
    key = sprintf('%d_%d_%.12g_%.12g', d.n, substeps, d.ka, d.kb);

    if ~isKey(cache, key)
        import casadi.*
        y = SX.sym('y', 7);
        u = SX.sym('u', 3);
        dt = u(3);
        k1 = rhs(y, u, d);
        k2 = rhs(y + dt * k1 / 2, u, d);
        k3 = rhs(y + dt * k2 / 2, u, d);
        k4 = rhs(y + dt * k3, u, d);
        one = Function('flow', {y, u}, {y + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6});
        cache(key) = one.mapaccum((d.n - 1) * substeps);
    end
    q = reshape(z(2:end), d.n, 9);
    u = repelem(q(1:end - 1, 5:6)', 1, substeps);
    u(3, :) = z(1) / substeps;
    f = cache(key);
    x = [q(1, [1:4, 7:9]); full(f(q(1, [1:4, 7:9])', u))'];
end

function f = rhs(x, u, d)
    k = d.ka - d.kb * u(2) ^ 2;
    v = x(4);
    a = x(5);
    b = x(6);
    g = x(7);
    f = [v * cos(a) * cos(b); v * sin(a) * cos(b); v * sin(b); u(1); k * v * sin(g) / cos(b); k * v * cos(g); u(2)];
end

function s = SelfCheck(x, d, c)
    x = x(:, 1:3) * 1000 / d.scale;
    v = diff(x);
    len = sqrt(sum(v .^ 2, 2));
    arc = [0; cumsum(len)];
    mid = (x(1:end - 1, :) + x(2:end, :)) / 2;
    radius = 2 * d.needle_radius * 1000 / d.scale;
    D = sqrt(max(0, sum(mid .^ 2, 2) + sum(mid .^ 2, 2)' - 2 * (mid * mid'))) - len / 2 - len' / 2;
    gap = arc(1:end - 1)' - arc(2:end);
    mask = gap >= c.self_arc_mm;
    D(~mask) = Inf;
    [ii, jj] = find(D < radius);
    actual = Inf;

    for k = 1:numel(ii)
        dist = segdist(x(ii(k), :), x(ii(k) + 1, :), x(jj(k), :), x(jj(k) + 1, :));
        actual = min(actual, dist);
        D(ii(k), jj(k)) = dist;
    end
    s = struct('feasible', actual >= radius, 'lower_bound_mm', min(D(:)), 'tested_pairs', numel(ii), 'arc_exclusion_mm', c.self_arc_mm, 'method', 'segment_pairs');
end

function d = segdist(p1, p2, q1, q2)
    u = p2 - p1;
    v = q2 - q1;
    w = p1 - q1;
    a = dot(u, u);
    b = dot(u, v);
    cc = dot(v, v);
    dd = dot(u, w);
    e = dot(v, w);
    points = [0, max(0, min(1, e / max(cc, eps))); 1, max(0, min(1, (e + b) / max(cc, eps))); max(0, min(1, -dd / max(a, eps))), 0; max(0, min(1, (b - dd) / max(a, eps))), 1];
    den = a * cc - b * b;

    if den > 1e-18
        t = (b * e - cc * dd) / den;
        s = (a * e - b * dd) / den;
        if t >= 0 && t <= 1 && s >= 0 && s <= 1
            points(end + 1, :) = [t, s];
        end
    end
    delta = w + points(:, 1) .* u - points(:, 2) .* v;
    d = sqrt(min(sum(delta .^ 2, 2)));
end
