function v = ReplanValidate(z, d, c, mode, p, extra)
    if nargin < 6
        extra = struct();
    end
    v = struct('feasible', false, 'eq', Inf, 'ineq', Inf, 'terminal_mm', Inf, 'clearance_mm', -Inf, 'segment_clearance_mm', NaN, 'duration', Inf, 'length_mm', Inf, 'effort', Inf, 'smoothness', Inf, 'near_seconds', Inf, 'max_near_seconds', Inf, 'cost', Inf, 'legacy_cost', Inf);
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

    if strcmp(mode, 'online')
        state_ids = [1:4, 7:9];
        boundary = (q(1, state_ids) - p(state_ids)') ./ qs(state_ids);
    else
        boundary = [(q(1, 3) - d.start(3)) / qs(3), q(1, 4:6) ./ qs(4:6)];
        bound = max([bound, (abs(q(1, 1:2) - d.start(1:2)') - d.start_slack) ./ qs(1:2), -q(1, 8), q(1, 8) - acos(d.cos_limit), abs(q(1, 7)) - pi, abs(q(1, 9)) - pi]);
    end
    boundary = [boundary, q(end, 4:6) ./ qs(4:6)];
    v.terminal_mm = norm(q(end, 1:3) - d.goal') * 1000 / d.scale;

    if ~strcmp(mode, 'presolve')
        boundary = [boundary, (q(end, 1:3) - d.goal') ./ qs(1:3)];
    end
    v.eq = max(abs([e ./ es; boundary(:)]));
    centers = reshape(p(10:end), 3, []);
    clearance = inf(n, 1);
    sphere_violation = 0;
    seg = Inf;
    a = q(1:end - 1, 1:3);
    delta = diff(q(:, 1:3));

    for j = 1:size(centers, 2)
        radius = d.radii(j) + d.needle_radius;
        dist = sqrt(sum((q(:, 1:3) - centers(:, j)') .^ 2, 2));
        clearance = min(clearance, dist - radius);
        sphere_violation = max(sphere_violation, max((radius ^ 2 - dist .^ 2) / max(radius ^ 2, (.001 * d.scale) ^ 2)));
        if c.check_segments
            tau = max(0, min(1, sum((centers(:, j)' - a) .* delta, 2) ./ max(sum(delta .^ 2, 2), eps)));
            seg = min(seg, min(sqrt(sum((a + tau .* delta - centers(:, j)') .^ 2, 2)) - radius));
        end
    end
    if isfield(extra, 'box')
        bound = max([bound; reshape((extra.box(1, :) - q(:, 1:3)) / (.1 * d.scale), [], 1); reshape((q(:, 1:3) - extra.box(2, :)) / (.1 * d.scale), [], 1)]);
    end
    if isfield(extra, 'corridor')
        bound = max([bound; reshape((extra.corridor(:, 1:3) - q(:, 1:3)) / (.1 * d.scale), [], 1); reshape((q(:, 1:3) - extra.corridor(:, 4:6)) / (.1 * d.scale), [], 1)]);
    end
    v.ineq = max([bound; sphere_violation; d.cos_limit ^ 2 - cos(q(:, 8)) .^ 2]);
    v.clearance_mm = min(clearance) * 1000 / d.scale;

    if c.check_segments
        v.segment_clearance_mm = seg * 1000 / d.scale;
    end
    v.duration = (n - 1) * h;
    v.length_mm = sum(sqrt(sum(diff(q(:, 1:3)) .^ 2, 2))) * 1000 / d.scale;
    v.effort = h * sum((q(:, 5) / d.amax) .^ 2 + (q(:, 6) / d.wmax) .^ 2);
    v.smoothness = sum(diff(q(:, 5) / d.amax) .^ 2 + diff(q(:, 6) / d.wmax) .^ 2) / h;

    if strcmp(mode, 'online')
        v.smoothness = v.smoothness + ((q(1, 5) - p(5)) / d.amax) ^ 2 / h + ((q(1, 6) - p(6)) / d.wmax) ^ 2 / h;
    end
    v.cost = v.duration + c.effort_weight * v.effort + c.smooth_weight * v.smoothness;

    if isfield(c, 'attitude_weight') && c.attitude_weight > 0
        gap = cos(q(:, 8)) .^ 2 - d.cos_limit ^ 2;
        positive_gap = .5 * (gap + sqrt(gap .^ 2 + 1e-12));
        v.cost = v.cost - c.attitude_weight * h * sum(log(positive_gap / (1 - d.cos_limit ^ 2)));
    end
    v.feasible = v.eq <= c.eq_tol && v.ineq <= c.ineq_tol;

    if c.check_segments
        v.feasible = v.feasible && seg >= -1e-7 * d.scale;
    end
end
